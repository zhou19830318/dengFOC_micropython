DengFOC_micropython是基于灯哥开源的DengFOC库，使用 micropython 重构的FOC电机控制代码。
```python
from DengFOC import *
# 示例使用
if __name__ == '__main__':
    DFOC_Vbus(12.6)
    DFOC_enable()
    DFOC_M0_alignSensor(7, 1)
    
    while True:
        runFOC()
        DFOC_M0_setVelocity(1)  # 目标速度10rad/s
        time.sleep_ms(1)
```
