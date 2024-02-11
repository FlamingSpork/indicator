import sys
import time

import obd

# obd.logger.setLevel(obd.logging.DEBUG)

obdc = obd.OBD("/dev/rfcomm1", fast=False, start_low_power=True)
if obdc.status() == obd.OBDStatus.NOT_CONNECTED:
    sys.exit(4)

while True:
    speed = obdc.query(obd.commands.SPEED)

    mph = speed.value.to("mph")
    print(speed.value, mph)

    time.sleep(0.1)
