import time

from .adu import ADU, Lamps

adu = ADU("/dev/ttyUSB0")
adu.connect()
adu.set_speed(10)
time.sleep(3)
adu.set_speed(30)
time.sleep(3)
adu.speed_lamps(60)
adu.on_lamp(Lamps.YARD_10)
adu.on_lamp(Lamps.BYPASS)
time.sleep(5)
