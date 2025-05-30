import utime

def _constrain(amt, low, high):
    return max(low, min(amt, high))

class PIDController:
    def __init__(self, P, I, D, ramp, limit):
        self.P = P  # 比例增益(P环增益)
        self.I = I  # 积分增益（I环增益）
        self.D = D  # 微分增益（D环增益）
        self.output_ramp = ramp  # PID控制器加速度限幅
        self.limit = limit  # PID控制器输出限幅
        
        self.error_prev = 0.0  # 最后的跟踪误差值
        self.output_prev = 0.0  # 最后一个 pid 输出值
        self.integral_prev = 0.0  # 最后一个积分分量值
        self.timestamp_prev = utime.ticks_us()  # 上次执行时间戳

    def __call__(self, error):
        # 计算两次循环中间的间隔时间
        timestamp_now = utime.ticks_us()
        Ts = utime.ticks_diff(timestamp_now, self.timestamp_prev) * 1e-6  # 转换为秒
        
        if Ts <= 0 or Ts > 0.5: # 如果时间间隔无效或过长
            Ts = 1e-3  # 默认1ms

        # P环
        proportional = self.P * error
        
        # Tustin 散点积分（I环）
        integral = self.integral_prev + self.I * Ts * 0.5 * (error + self.error_prev)
        integral = _constrain(integral, -self.limit, self.limit)
        
        # D环（微分环节）
        # 避免在Ts非常小的时候除以0或得到过大的微分值
        if Ts > 1e-9: # 避免除以零或极小值
            derivative = self.D * (error - self.error_prev) / Ts
        else:
            derivative = 0.0

        # 将P,I,D三环的计算值加起来
        output = proportional + integral + derivative
        output = _constrain(output, -self.limit, self.limit)

        if self.output_ramp > 0:
            if Ts > 1e-9: # 避免除以零或极小值
                # 对PID的变化速率进行限制
                output_rate = (output - self.output_prev) / Ts
                if output_rate > self.output_ramp:
                    output = self.output_prev + self.output_ramp * Ts
                elif output_rate < -self.output_ramp:
                    output = self.output_prev - self.output_ramp * Ts
            # else: if Ts is too small, output ramp might not be effective or could be skipped

        # 保存值（为了下一次循环）
        self.integral_prev = integral
        self.output_prev = output
        self.error_prev = error
        self.timestamp_prev = timestamp_now
        
        return output

# Example Usage:
if __name__ == '__main__':
    # 初始化PID控制器参数
    # PIDController(P, I, D, ramp, limit)
    pid = PIDController(P=1.0, I=0.5, D=0.1, ramp=1000, limit=10)

    target_value = 5.0
    current_value = 0.0

    print("Time (s) | Error   | Output  | Current Value")
    print("-------------------------------------------------")

    start_time = utime.ticks_ms()

    for i in range(100):
        error = target_value - current_value
        control_output = pid(error)
        
        # 模拟系统响应，这里简单地将输出的一部分加到当前值上
        # 在实际应用中，这将是物理系统的响应
        current_value += control_output * 0.01 # 假设一个简单的系统增益和时间步长效应
        current_value = _constrain(current_value, -20, 20) # 限制当前值的范围

        elapsed_time_s = utime.ticks_diff(utime.ticks_ms(), start_time) / 1000.0
        print(f"{elapsed_time_s:8.3f} | {error:7.3f} | {control_output:7.3f} | {current_value:7.3f}")
        
        utime.sleep_ms(10) # 模拟控制周期

        if abs(error) < 0.01:
            print("\nTarget reached.")
            break
