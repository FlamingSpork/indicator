import collections
import functools
import json
import math
import sys
import time
from decimal import Decimal

import obd
import paho.mqtt
import paho.mqtt.client
from lxml import etree
from numpy import arccos, array, cross, dot, pi
from numpy.linalg import norm

from ..adu import ADU

LOOKUP_PREC = 3
NEIGHBORS = [(x, y) for x in range(-2, 3) for y in range(-2, 3)]


nodes = {}
squares = collections.defaultdict(set)


def timed(desc):
    def dec(__f):
        @functools.wraps(__f)
        def decorated(*a, **k):
            start = time.time()
            result = __f(*a, **k)
            length = time.time() - start

            if length < 1:
                print(f"{desc} took {round(length * 1_000, 3)}ms", file=sys.stderr)
            else:
                print(f"{desc} took {round(length, 3)}s", file=sys.stderr)

            return result

        return decorated

    return dec


def degrees_to_radians(degrees):
    return float(degrees) * math.pi / 180


def dist(a, b, r=6_378_137):
    a = [degrees_to_radians(i) for i in a]
    b = [degrees_to_radians(i) for i in b]

    dlon = abs(a[0] - b[0])

    central_angle = math.atan2(
        math.sqrt(
            (math.cos(b[1]) * math.sin(dlon)) ** 2
            + (
                (math.cos(a[1]) * math.sin(b[1]))
                - (math.sin(a[1]) * math.cos(b[1]) * math.cos(dlon))
            )
            ** 2
        ),
        (math.sin(a[1]) * math.sin(b[1]))
        + (math.cos(a[1]) + math.cos(b[1])) * math.cos(dlon),
    )

    return 2 * r * central_angle


def segment_dist(A, B, P):
    # https://gist.github.com/nim65s/5e9902cd67f094ce65b0
    if all(A == P) or all(B == P):
        return 0

    if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > pi / 2:
        return norm(P - A)

    if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > pi / 2:
        return norm(P - B)

    return norm(cross(A - B, A - P)) / norm(B - A)


class Point:
    def __init__(self, lat, lon):
        self.lat = Decimal(lat)
        self.lon = Decimal(lon)

    def np_arr(self):
        return array([float(self.lon * 10_000), float(self.lat * 10_000)])

    def grid_square(self):
        return (int(self.lon * 10**LOOKUP_PREC), int(self.lat * 10**LOOKUP_PREC))

    @functools.cache
    def nearby_nodes(self):
        thisx, thisy = self.grid_square()

        result = []

        for offx, offy in NEIGHBORS:
            result.extend(squares[(thisx + offx, thisy + offy)])

        result = [(dist((self.lon, self.lat), (k.lon, k.lat)), k) for k in result]
        result.sort()

        result = [(d, k) for (d, k) in result]  # could filter by distance here
        return result

    @functools.cache
    def nearby_ways(self):
        seen_ways = {}
        done_ways = set()

        result = []

        for _, node in self.nearby_nodes():
            for way in node.ways:
                if way in done_ways:
                    continue

                if way in seen_ways:
                    last = seen_ways[way]
                    dist = segment_dist(node.np_arr(), last.np_arr(), self.np_arr())
                    result.append((dist, way))
                    done_ways.add(way)

                else:
                    seen_ways[way] = node

        result.sort(key=lambda k: k[0])
        return result

    def __repr__(self):
        return f"Point({self.lat}, {self.lon})"

    def __hash__(self):
        return hash((self.lon, self.lat))


class Node(Point):
    def __init__(self, lat, lon, nid):
        self.nid = nid
        self.ways = set()

        super().__init__(lat, lon)

    def save(self):
        nodes[self.nid] = self
        squares[self.grid_square()].add(self)

    def __hash__(self):
        return hash(self.nid)

    def __repr__(self):
        return f"Node({self.lat}, {self.lon}, {self.nid!r})"


class Way:
    def __init__(self, osm):
        self.wid = osm.get("id")
        self.nodes = set()
        self.tags = {}

        for el in osm:
            if el.tag == "nd":
                ref = el.get("ref")
                if ref in nodes:
                    self.nodes.add(nodes[ref])

            elif el.tag == "tag":
                self.tags[el.get("k")] = el.get("v")

    def update_nodes(self):
        for node in self.nodes:
            node.ways.add(self)

    def __hash__(self):
        return hash(self.wid)

    def __repr__(self):
        return f"Way<{self.wid=}, {len(self.nodes)=}, {self.tags!r}>"


@timed("snap to way")
def snap_to_way(pts):
    ways = collections.Counter()
    way_dists = collections.defaultdict(list)

    min_dists = []
    for pt in pts:
        near_ways = pt.nearby_ways()
        if near_ways:
            min_dists.append(near_ways[0][0])

    threshold = max(min_dists) * 2
    for pt in pts:
        for dist, way in pt.nearby_ways():
            if dist > threshold:
                continue

            ways[way] += 1
            way_dists[way].append(dist)

    result = []

    most_frequent_count = max(ways.values())
    frequent_ways = [
        way for (way, count) in ways.items() if count == most_frequent_count
    ]

    for way in frequent_ways:
        result.append((sum(dist**2 for dist in way_dists[way]), way))

    result.sort(key=lambda k: k[0])
    return result


@timed("build tree")
def build_tree():
    with open("driver/osm/highways.osm") as f:
        et = etree.parse(f)

        for el in et.getroot():
            if el.tag == "relation":
                continue

            elif el.tag == "node":
                it = dict(el.items())
                Node(it["lat"], it["lon"], it["id"]).save()

            elif el.tag == "way":
                Way(el).update_nodes()


points = []
adu = ADU("/dev/ttyUSB0")


def on_position(_client, _userdata, msg):
    data = json.loads(msg.payload.decode())
    if data["_type"] != "location":
        return

    if points:
        if dist((points[-1].lon, points[-1].lat), (data["lon"], data["lat"])) <= 10:
            return

    points.append(Point(data["lat"], data["lon"]))

    if len(points) > 6:
        points.pop(0)

    ways = snap_to_way(points)
    if ways:
        way = ways[0][1]
        name = way.tags.get("name", "(none)")
        ref = way.tags.get("ref", "(none)")
        speed = way.tags.get("maxspeed")

        if speed:
            speed = int(speed.split()[0])
            adu.speed_lamps(speed, exact=True)
        else:
            adu.no_speed_lamps()

        print("Found way")
        print("WAY ID:", way.wid)
        print("NAME:", name)
        print("REF:", ref)
        print("SPEED:", speed)
        print("---")


def on_connect(client, userdata, _flags, _rc):
    print("connected to mqtt")
    client.subscribe("owntracks/tris/+")


if __name__ == "__main__":
    mqtt = paho.mqtt.client.Client(paho.mqtt.client.CallbackAPIVersion.VERSION1)
    mqtt.on_connect = on_connect
    mqtt.on_message = on_position

    build_tree()

    mqtt.connect("trisfyi", 1883)
    adu.connect()

    obdc = obd.OBD("/dev/rfcomm1", fast=False, start_low_power=True)
    if obdc.status() == obd.OBDStatus.NOT_CONNECTED:
        sys.exit(4)

    speed = obdc.query(obd.commands.SPEED)
    print("obd speed is", speed)

    while True:
        adu.on_lamp(18)
        adu.on_lamp(19)
        speed = obdc.query(obd.commands.SPEED)

        if speed.value is None:
            print("obd reconnect")
            obdc = obd.OBD("/dev/rfcomm1", fast=False, start_low_power=True)
            continue

        adu.set_speed(int(round(speed.value.to("mph").m)))
        mqtt.loop()

        time.sleep(0.1)
