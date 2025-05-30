import utime

class LowPassFilter:
    def __init__(self, time_constant):
        self.Tf = time_constant  # 低通滤波时间常数
        self.y_prev = 0.0        # 上一个循环中的过滤后的值
        self.timestamp_prev = utime.ticks_us()  # 最后执行时间戳

    def __call__(self, x):
        timestamp = utime.ticks_us()
        dt = utime.ticks_diff(timestamp, self.timestamp_prev) * 1e-6  # 转换为秒

        if dt < 0.0:
            # 时间戳回绕或错误，重置dt为一个较小的值
            dt = 1e-3
        elif dt > 0.3:  # 如果时间间隔过长，则重置滤波器状态
            self.y_prev = x
            self.timestamp_prev = timestamp
            return x

        alpha = self.Tf / (self.Tf + dt)
        y = alpha * self.y_prev + (1.0 - alpha) * x
        
        self.y_prev = y
        self.timestamp_prev = timestamp
        return y

# Example Usage:
if __name__ == '__main__':
    # 创建一个时间常数为0.01秒的低通滤波器
    lpf = LowPassFilter(time_constant=0.01)

    # 模拟一些输入数据
    input_signal = [1.0, 1.1, 0.9, 1.0, 1.05, 0.95, 2.0, 2.1, 1.9, 2.0]
    
    print("Input -> Output")
    for val in input_signal:
        filtered_val = lpf(val) # 调用滤波器
        print(f"{val:.2f} -> {filtered_val:.2f}")
        utime.sleep_ms(10) # 模拟10ms的采样间隔

    print("\nSimulating a long delay between samples:")
    utime.sleep_ms(500) # 500ms delay
    val_after_delay = 5.0
    filtered_val_after_delay = lpf(val_after_delay)
    print(f"{val_after_delay:.2f} -> {filtered_val_after_delay:.2f} (after long delay, filter should reset or adapt quickly)")
