import time

class LowPassFilter:
    def __init__(self, Tf):
        """
        初始化低通滤波器
        :param Tf: 低通滤波时间常数
        """
        self.Tf = Tf  # 低通滤波时间常数
        self.y_prev = 0.0  # 上一个循环中的过滤后的值
        self.timestamp_prev = time.ticks_us()  # 最后执行时间戳（微秒）

    def __call__(self, x):
        """
        执行低通滤波
        :param x: 输入值
        :return: 滤波后的输出值
        """
        timestamp = time.ticks_us()  # 获取当前时间戳（微秒）
        # 计算时间差（转换为秒）
        dt = time.ticks_diff(timestamp, self.timestamp_prev) * 1e-6

        # 处理时间差的边界条件
        if dt < 0.0:
            dt = 1e-3  # 最小时间差设为 1ms
        elif dt > 0.3:
            self.y_prev = x
            self.timestamp_prev = timestamp
            return x

        # 计算滤波系数 alpha 并进行滤波
        alpha = self.Tf / (self.Tf + dt)
        y = alpha * self.y_prev + (1.0 - alpha) * x
        self.y_prev = y
        self.timestamp_prev = timestamp
        return y
'''
# 示例用法
filter = LowPassFilter(0.1)  # Tf = 0.1
filtered_value = filter(10.0)  # 输入值 10.0，获取滤波后的值
print(filtered_value)
'''
