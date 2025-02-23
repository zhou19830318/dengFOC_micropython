import machine
import math
import time

_2PI = 6.28318530718

class Sensor_AS5600:
    def __init__(self, Mot_Num):
        self._Mot_Num = Mot_Num
        self.angle_prev = 0
        self.angle_prev_ts = 0
        self.vel_angle_prev = 0
        self.vel_angle_prev_ts = 0
        self.full_rotations = 0
        self.vel_full_rotations = 0
        self.wire = None

    def Sensor_init(self, I2C_NUM, scl_pin, sda_pin):
        # 初始化 I2C，MicroPython 使用 machine.I2C
        self.wire = machine.I2C(I2C_NUM, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
        time.sleep_ms(500)  # delay(500)
        self.getSensorAngle()
        time.sleep_us(1)  # delayMicroseconds(1)
        self.vel_angle_prev = self.getSensorAngle()
        self.vel_angle_prev_ts = time.ticks_us()
        time.sleep_ms(1)  # delay(1)
        self.getSensorAngle()
        time.sleep_us(1)  # delayMicroseconds(1)
        self.angle_prev = self.getSensorAngle()
        self.angle_prev_ts = time.ticks_us()

    def getSensorAngle(self):
        angle_reg_msb = 0x0C
        # I2C 读取数据
        readArray = self.wire.readfrom_mem(0x36, angle_reg_msb, 2)
        readValue = 0

        _bit_resolution = 12
        _bits_used_msb = 11 - 7  # 4
        cpr = 2 ** _bit_resolution  # pow(2, _bit_resolution)
        lsb_used = _bit_resolution - _bits_used_msb  # 8

        lsb_mask = (2 << lsb_used) - 1  # 模拟位操作
        msb_mask = (2 << _bits_used_msb) - 1

        readValue = (readArray[1] & lsb_mask)
        readValue += ((readArray[0] & msb_mask) << lsb_used)
        return (readValue / cpr) * _2PI

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
        return self.full_rotations * _2PI + self.angle_prev

    def getVelocity(self):
        # 计算采样时间
        Ts = (self.angle_prev_ts - self.vel_angle_prev_ts) * 1e-6
        # 快速修复奇怪的情况（如时间戳溢出）
        if Ts <= 0:
            Ts = 1e-3
        # 速度计算
        vel = ((self.full_rotations - self.vel_full_rotations) * _2PI + 
               (self.angle_prev - self.vel_angle_prev)) / Ts
        # 保存变量以待将来使用
        self.vel_angle_prev = self.angle_prev
        self.vel_full_rotations = self.full_rotations
        self.vel_angle_prev_ts = self.angle_prev_ts
        return vel
'''
# 示例用法
if __name__ == "__main__":
    sensor = Sensor_AS5600(1)  # Mot_Num = 1
    sensor.Sensor_init(0, 18, 19)  # 示例引脚，根据硬件调整 scl_pin, sda_pin
    while True:
        sensor.Sensor_update()
        print("Angle:", sensor.getAngle())
        print("Velocity:", sensor.getVelocity())
        time.sleep_ms(100)
'''
