from DengFOC import *
import time

    
DFOC_Vbus(12.6)
DFOC_enable()
DFOC_M0_alignSensor(7, 1)

while True:
    runFOC()
    DFOC_M0_setVelocity(10)  # 目标速度10rad/s
    time.sleep_ms(1)
