import time

class LowPassFilter:
    def __init__(self, time_constant):
        self.Tf = time_constant
        self.y_prev = 0.0
        self.timestamp_prev = time.ticks_us()
    
    def update(self, x):
        timestamp = time.ticks_us()
        dt = time.ticks_diff(timestamp, self.timestamp_prev) * 1e-6  # 转换为秒
        
        # 处理异常时间差
        if dt < 0.0:
            dt = 1e-3  # 负时间差时设为1ms
        elif dt > 0.3:
            self._reset(x, timestamp)
            return x
        
        # 计算滤波系数
        alpha = self.Tf / (self.Tf + dt)
        
        # 应用滤波公式
        y = alpha * self.y_prev + (1.0 - alpha) * x
        
        # 更新状态
        self.y_prev = y
        self.timestamp_prev = timestamp
        
        return y
    
    def _reset(self, x, timestamp):
        """重置滤波器状态"""
        self.y_prev = x
        self.timestamp_prev = timestamp

"""
示例用法：
lpf = LowPassFilter(time_constant=0.1)  # 创建时间常数为0.1秒的滤波器
filtered_value = lpf.update(10.0)       # 输入新值并获取滤波结果
"""
