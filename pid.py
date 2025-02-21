from time import ticks_us, ticks_diff

# 定义 _constrain 函数，等同于 C++ 中的宏定义
def _constrain(amt, low, high):
    if amt < low:
        return low
    elif amt > high:
        return high
    return amt

class PIDController:
    def __init__(self, P, I, D, ramp, limit):
        # 初始化参数，与 C++ 版本一致
        self.P = P                    # 比例增益 (P 环增益)
        self.I = I                    # 积分增益 (I 环增益)
        self.D = D                    # 微分增益 (D 环增益)
        self.output_ramp = ramp       # PID 控制器加速度限幅
        self.limit = limit            # PID 控制器输出限幅
        self.error_prev = 0.0         # 最后的跟踪误差值
        self.output_prev = 0.0        # 最后一个 PID 输出值
        self.integral_prev = 0.0      # 最后一个积分分量值
        self.timestamp_prev = ticks_us()  # 上次执行时间戳（使用 ticks_us）

    def __call__(self, error):
        # 计算两次循环中间的间隔时间
        timestamp_now = ticks_us()
        Ts = ticks_diff(timestamp_now, self.timestamp_prev) * 1e-6  # 转换为秒
        #print(Ts)
        if Ts <= 0 or Ts > 0.5:
            Ts = 1e-3  # 默认 1ms，避免异常
        
        # P 环
        proportional = self.P * error
        
        # Tustin 散点积分 (I 环)
        integral = self.integral_prev + self.I * Ts * 0.5 * (error + self.error_prev)
        integral = _constrain(integral, -self.limit, self.limit)
        
        # D 环 (微分环节)
        derivative = self.D * (error - self.error_prev) / Ts
        
        # 将 P, I, D 三环的计算值加起来
        output = proportional + integral + derivative
        output = _constrain(output, -self.limit, self.limit)
        
        # 对 PID 输出变化速率进行限制
        if self.output_ramp > 0:
            output_rate = (output - self.output_prev) / Ts
            if output_rate > self.output_ramp:
                output = self.output_prev + self.output_ramp * Ts
            elif output_rate < -self.output_ramp:
                output = self.output_prev - self.output_ramp * Ts
        
        # 保存值 (为了下一次循环)
        self.integral_prev = integral
        self.output_prev = output
        self.error_prev = error
        self.timestamp_prev = timestamp_now
        
        return output
'''
# 创建 PID 控制器实例
pid = PIDController(P=1.0, I=0.1, D=0.05, ramp=10.0, limit=100.0)

# 计算 PID 输出
error = 5.0  # 示例误差
output = pid(error)
print("PID Output:", output)
'''
