from machine import I2C, Pin
import math
import time
import struct

_2PI = 6.28318530718

class Sensor_AS5600:
    def __init__(self, Mot_Num):
        self._Mot_Num = Mot_Num  # 使得 Mot_Num 可以统一在该文件调用
        self.wire = None
        self.angle_prev = 0
        self.angle_prev_ts = 0
        self.full_rotations = 0
        self.vel_angle_prev = 0
        self.vel_angle_prev_ts = 0
        self.vel_full_rotations = 0
    
    def getSensorAngle(self):
        angle_reg_msb = 0x0C
        
        self.wire.writeto(0x36, bytes([angle_reg_msb]))
        readArray = self.wire.readfrom(0x36, 2)
        
        _bit_resolution = 12
        _bits_used_msb = 11-7
        cpr = math.pow(2, _bit_resolution)
        lsb_used = _bit_resolution - _bits_used_msb
        
        lsb_mask = (2 << lsb_used) - 1
        msb_mask = (2 << _bits_used_msb) - 1
        
        readValue = (readArray[1] & lsb_mask)
        readValue += ((readArray[0] & msb_mask) << lsb_used)
        
        return (readValue / cpr) * _2PI
    
    def Sensor_init(self, _wire):
        self.wire = _wire
        # MicroPython I2C 已初始化，不需要begin()
        time.sleep_ms(500)
        
        self.getSensorAngle()
        time.sleep_us(1)
        
        self.vel_angle_prev = self.getSensorAngle()
        self.vel_angle_prev_ts = time.ticks_us()
        
        time.sleep_ms(1)
        
        self.getSensorAngle()
        time.sleep_us(1)
        
        self.angle_prev = self.getSensorAngle()
        self.angle_prev_ts = time.ticks_us()
    
    def Sensor_update(self):
        val = self.getSensorAngle()
        self.angle_prev_ts = time.ticks_us()
        d_angle = val - self.angle_prev
        
        # 圈数检测
        if abs(d_angle) > (0.8 * _2PI):
            self.full_rotations += -1 if d_angle > 0 else 1
            
        self.angle_prev = val
    
    def getMechanicalAngle(self):
        return self.angle_prev
    
    def getAngle(self):
        return float(self.full_rotations) * _2PI + self.angle_prev
    
    def getVelocity(self):
        # 计算采样时间
        Ts = (self.angle_prev_ts - self.vel_angle_prev_ts) * 1e-6
        
        # 快速修复奇怪的情况（微溢出）
        if Ts <= 0:
            Ts = 1e-3
            
        # 速度计算
        vel = ((float(self.full_rotations - self.vel_full_rotations) * _2PI + 
                (self.angle_prev - self.vel_angle_prev)) / Ts)
                
        # 保存变量以待将来使用
        self.vel_angle_prev = self.angle_prev
        self.vel_full_rotations = self.full_rotations
        self.vel_angle_prev_ts = self.angle_prev_ts
        
        return vel
######################################################
# 初始化I2C - 使用适合你的开发板的引脚
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# 创建AS5600传感器实例
sensor = Sensor_AS5600(1)  # 电机编号参数

# 初始化传感器
sensor.Sensor_init(i2c)

# 使用传感器
while True:
    sensor.Sensor_update()
    angle = sensor.getAngle()
    velocity = sensor.getVelocity()
    print(f"Angle: {angle}, Velocity: {velocity}")
    time.sleep(0.1)
