from machine import I2C, Pin

class AS5600:
    def __init__(self, i2c=None, scl_pin=18, sda_pin=19, freq=400000, address=0x36):
        self._raw_ang_hi = 0x0C
        self._raw_ang_lo = 0x0D
        self._ams5600_Address = address
        self.full_rotations = 0
        self.angle_prev = 0.0
        
        if i2c is None:
            self.i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        else:
            self.i2c = i2c

    def _read_two_bytes(self, reg_hi, reg_lo):
        """读取两个寄存器的值并组合为16位数值"""
        try:
            low = self.i2c.readfrom_mem(self._ams5600_Address, reg_lo, 1)[0]
            high = self.i2c.readfrom_mem(self._ams5600_Address, reg_hi, 1)[0]
            return (high << 8) | low
        except OSError:
            return -1

    def getRawAngle(self):
        """获取原始角度数值（0-4095）"""
        return self._read_two_bytes(self._raw_ang_hi, self._raw_ang_lo)

    def getAngle_Without_track(self):
        """获取单圈弧度值（0-2π）"""
        raw = self.getRawAngle()
        return raw * (3.141592653589793 * 2) / 4096  # 精确计算弧度值

    def getAngle(self):
        """获取累积多圈角度值（单位：弧度）"""
        val = self.getAngle_Without_track()
        d_angle = val - self.angle_prev
        
        # 全圈数检测逻辑
        if abs(d_angle) > 0.8 * 6.28318530718:
            self.full_rotations += -1 if (d_angle > 0) else 1
        
        self.angle_prev = val
        return self.full_rotations * 6.28318530718 + val

# 使用示例
if __name__ == "__main__":
    sensor = AS5600()
    while True:
        print("Current Angle (rad):", sensor.getAngle())
