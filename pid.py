import time

class PIDController:
    def __init__(self, P, I, D, output_ramp, limit):
        self.P = P
        self.I = I
        self.D = D
        self.output_ramp = output_ramp
        self.limit = limit
        
        self.error_prev = 0.0
        self.output_prev = 0.0
        self.integral_prev = 0.0
        self.timestamp_prev = time.ticks_us()
    
    def _constrain(self, amt, low, high):
        return max(min(amt, high), low)
    
    def __call__(self, error):
        # 计算时间间隔（秒）
        timestamp_now = time.ticks_us()
        Ts = time.ticks_diff(timestamp_now, self.timestamp_prev) * 1e-6
        
        # 处理异常时间间隔
        if Ts <= 0 or Ts > 0.5:
            Ts = 1e-3
        
        # 计算P项
        proportional = self.P * error
        
        # 计算I项（Tustin积分法）
        integral = self.integral_prev + self.I * Ts * 0.5 * (error + self.error_prev)
        integral = self._constrain(integral, -self.limit, self.limit)
        
        # 计算D项
        derivative = self.D * (error - self.error_prev) / Ts
        
        # 计算总输出
        output = proportional + integral + derivative
        output = self._constrain(output, -self.limit, self.limit)
        
        # 输出变化率限制
        if self.output_ramp > 0:
            output_rate = (output - self.output_prev) / Ts
            if output_rate > self.output_ramp:
                output = self.output_prev + self.output_ramp * Ts
            elif output_rate < -self.output_ramp:
                output = self.output_prev - self.output_ramp * Ts
            output = self._constrain(output, -self.limit, self.limit)
        
        # 保存状态
        self.integral_prev = integral
        self.output_prev = output
        self.error_prev = error
        self.timestamp_prev = timestamp_now
        
        return output
