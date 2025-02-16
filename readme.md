DengFOC_micropython库是基于灯哥开源的DengFOC库，使用 micropython 重构的开源 FOC 库。

#as5600模块应用举例：
##1. 使用不同 I2C 地址
如果多个 AS5600 编码器连接到同一个 I2C 总线，但每个编码器的地址不同（通过硬件配置），可以通过指定不同的 address 参数来实例化多个对象。

'''python'''
from machine import I2C, Pin
from as5600 import AS5600

###初始化 I2C 总线
i2c = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)

###实例化多个 AS5600 编码器（假设地址分别为 0x36 和 0x37）
encoder1 = AS5600(i2c=i2c, address=0x36)
encoder2 = AS5600(i2c=i2c, address=0x37)

###读取角度值
while True:
    print("Encoder 1 Angle:", encoder1.getAngle())
    print("Encoder 2 Angle:", encoder2.getAngle())
##使用不同的 I2C 总线
如果多个 AS5600 编码器的地址相同（无法更改），可以将它们连接到不同的 I2C 总线上，然后分别实例化。

'''python'''
from machine import I2C, Pin
from as5600 import AS5600

###初始化两个 I2C 总线
i2c1 = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)  # 总线 1
i2c2 = I2C(1, scl=Pin(25), sda=Pin(26), freq=400000)  # 总线 2

###实例化多个 AS5600 编码器
encoder1 = AS5600(i2c=i2c1, address=0x36)  # 连接到总线 1
encoder2 = AS5600(i2c=i2c2, address=0x36)  # 连接到总线 2

###读取角度值
while True:
    print("Encoder 1 Angle:", encoder1.getAngle())
    print("Encoder 2 Angle:", encoder2.getAngle())
    
##动态检测 I2C 设备
如果不知道具体的 I2C 地址，可以通过扫描 I2C 总线动态检测连接的设备，然后实例化对应的编码器。

'''python'''
from machine import I2C, Pin
from as5600 import AS5600

###初始化 I2C 总线
i2c = I2C(0, scl=Pin(18), sda=Pin(19), freq=400000)

###扫描 I2C 总线上的设备
devices = i2c.scan()
print("Detected I2C devices:", [hex(addr) for addr in devices])

###实例化所有检测到的 AS5600 编码器
encoders = []
for addr in devices:
    if addr == 0x36 or addr == 0x37:  # 只实例化已知的 AS5600 地址
        encoders.append(AS5600(i2c=i2c, address=addr))

###读取角度值
while True:
    for i, encoder in enumerate(encoders):
        print(f"Encoder {i+1} Angle:", encoder.getAngle())
