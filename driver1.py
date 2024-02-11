import time

import serial


class Lamps:
    STOP = 0
    LED_0 = 1
    LED_10 = 2
    LED_15 = 3
    LED_18 = 4
    LED_25 = 5
    LED_40 = 6
    LED_50 = 7
    LED_55 = 8
    LED_60 = 9
    YARD_10 = 10
    MAN_RL = 11
    UMAN_RL = 12
    BYPASS = 13
    LW_40 = 14
    W_40 = 15
    G_ATO = 16
    Y_MAN = 17
    LOC = 18
    REMOTE = 19


class Buttons:
    BRAKE_DEPART = 0
    ATO_MAN = 1


lamp_map = {
    0: Lamps.LED_0,
    10: Lamps.LED_10,
    15: Lamps.LED_15,
    18: Lamps.LED_18,
    25: Lamps.LED_25,
    40: Lamps.LED_40,
    50: Lamps.LED_50,
    55: Lamps.LED_55,
    60: Lamps.LED_60,
}


class ADU:
    def __init__(self, port):
        self.conn = serial.Serial(port, 9600)
        self.calib = (2.16, 1.03, 0.03499)

    def send(self, cmd, arg):
        self.conn.reset_input_buffer()
        self.conn.write(bytes([ord(cmd), arg, 0x20]))
        return self.conn.read()

    def pwm_value(self, mph):
        mph += mph * self.calib[2]
        return round(self.calib[0] * mph ** self.calib[1])


def lamps(x):
    for speed, pin in lamp_map.items():
        if x >= speed:
            s.write(bytes([ord("+"), pin, 0x20]))
        else:
            s.write(bytes([ord("-"), pin, 0x20]))


s = serial.Serial("/dev/ttyUSB0", 9600)

if False:
    while True:
        time.sleep(3)
        for i in range(0, 81):
            s.write(bytes([0x2F, pwm_value(i), 0x20]))
            lamps(i)
            time.sleep(0.15)

        time.sleep(3)
        for i in range(80, 0, -1):
            s.write(bytes([0x2F, pwm_value(i), 0x20]))
            lamps(i)
            time.sleep(0.15)


while True:
    n = int(input())
    if n == -1:
        break
    s.write(bytes([0x2F, pwm_value(n), 0x20]))
    lamps(n)

s.close()
