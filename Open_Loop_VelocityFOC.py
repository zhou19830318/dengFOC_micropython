# 灯哥开源，转载请著名出处
# 仅在DengFOC上测试过
import math
from machine import Pin, PWM
import time

class Motor:
    def __init__(self, pwm_pins, enable_pin, pole_pairs, voltage=7.4):
        """
        电机控制器类
        :param pwm_pins: PWM引脚列表 [A相, B相, C相]
        :param enable_pin: 使能引脚
        :param pole_pairs: 电机极对数
        :param voltage: 电源电压
        """
        # 硬件引脚初始化
        self.pwm = [PWM(Pin(pin)) for pin in pwm_pins] # 初始化PWM对象列表，用于控制电机三相
        self.enable_pin = Pin(enable_pin, Pin.OUT) # 初始化使能引脚为输出模式

        # 电机参数
        self.pole_pairs = pole_pairs # 设置电机极对数
        self.voltage = voltage # 设置电源电压

        # 状态变量
        self.shaft_angle = 0.0 # 初始化电机轴角度为0
        self.open_loop_timestamp = 0 # 初始化开环控制时间戳
        self.zero_electric_angle = 0.0 # 初始化零电角度，用于FOC对齐

        # PWM参数
        self.pwm_freq = 30000 # 设置PWM频率为30kHz
        self._init_pwm() # 调用内部方法初始化PWM

        # 初始化使能
        self.enable() # 默认使能电机驱动器

    def _init_pwm(self):
        """初始化PWM输出"""
        for p in self.pwm: # 遍历所有PWM引脚对象
            p.freq(self.pwm_freq) # 设置PWM频率

    def enable(self):
        """使能电机驱动器"""
        self.enable_pin.value(1) # 设置使能引脚为高电平，使能驱动器
        print("电机已使能") # 打印电机已使能信息

    def disable(self):
        """禁用电机驱动器"""
        self.enable_pin.value(0) # 设置使能引脚为低电平，禁用驱动器
        print("电机已禁用") # 打印电机已禁用信息

    @staticmethod
    def _constrain(amt, low, high):
        """数值约束方法"""
        return max(min(high, amt), low) # 将数值限制在[low, high]范围内

    def _electrical_angle(self, shaft_angle):
        """计算电角度"""
        return shaft_angle * self.pole_pairs # 电角度 = 轴角度 * 极对数

    @staticmethod
    def _normalize_angle(angle):
        """角度归一化到[0, 2π]"""
        a = math.fmod(angle, 2 * math.pi) # 取角度对2π的余数
        return a + 2 * math.pi if a < 0 else a # 如果余数为负数，则加上2π使其归一化到[0, 2π]

    def _set_pwm(self, Ua, Ub, Uc):
        """设置三相PWM输出"""
        # 计算占空比
        dc_a = self._constrain(Ua / self.voltage, 0.0, 1.0) # 计算A相占空比，电压值除以电源电压，并约束在[0, 1]
        dc_b = self._constrain(Ub / self.voltage, 0.0, 1.0) # 计算B相占空比，电压值除以电源电压，并约束在[0, 1]
        dc_c = self._constrain(Uc / self.voltage, 0.0, 1.0) # 计算C相占空比，电压值除以电源电压，并约束在[0, 1]

        # 设置PWM输出
        self.pwm[0].duty_u16(int(dc_a * 65535)) # 设置A相PWM占空比，将[0, 1]范围映射到[0, 65535]
        self.pwm[1].duty_u16(int(dc_b * 65535)) # 设置B相PWM占空比，将[0, 1]范围映射到[0, 65535]
        self.pwm[2].duty_u16(int(dc_c * 65535)) # 设置C相PWM占空比，将[0, 1]范围映射到[0, 65535]

    def set_phase_voltage(self, Uq, Ud, angle_el):
        """设置相电压"""
        # 帕克逆变换
        angle_el = self._normalize_angle(angle_el + self.zero_electric_angle) # 归一化电角度，并加上零电角度偏移
        Ualpha = -Uq * math.sin(angle_el) # 计算Alpha轴电压分量
        Ubeta = Uq * math.cos(angle_el) # 计算Beta轴电压分量

        # 克拉克逆变换
        Ua = Ualpha + self.voltage / 2 # 计算A相电压，进行逆克拉克变换和电压偏移
        Ub = (math.sqrt(3)*Ubeta - Ualpha)/2 + self.voltage/2 # 计算B相电压，进行逆克拉克变换和电压偏移
        Uc = (-Ualpha - math.sqrt(3)*Ubeta)/2 + self.voltage/2 # 计算C相电压，进行逆克拉克变换和电压偏移

        self._set_pwm(Ua, Ub, Uc) # 设置三相PWM输出

    def velocity_openloop(self, target_velocity):
        """开环速度控制"""
        now_us = time.ticks_us() # 获取当前时间，单位为微秒

        # 计算时间间隔
        Ts = time.ticks_diff(now_us, self.open_loop_timestamp) * 1e-6 # 计算时间间隔，单位为秒
        if Ts <= 0 or Ts > 0.5: # 避免时间间隔异常
            Ts = 1e-3 # 如果时间间隔异常，则设置为默认值1ms

        # 更新轴角度
        self.shaft_angle = self._normalize_angle(
            self.shaft_angle + target_velocity * Ts # 根据目标速度和时间间隔更新轴角度
        )

        # 设置输出电压
        Uq = self.voltage / 3 # 设置Q轴电压，开环控制中Ud通常为0，Uq控制速度
        self.set_phase_voltage(Uq, 0, self._electrical_angle(self.shaft_angle)) # 设置相电压，进行SVPWM调制

        self.open_loop_timestamp = now_us # 更新开环控制时间戳
        return Uq # 返回Q轴电压值，用于调试或监控

# 使用示例
if __name__ == "__main__":
    # 初始化电机对象
    motor1 = Motor(
        pwm_pins=[32, 33, 25],  # A, B, C相引脚
        enable_pin=22,         # 使能引脚
        pole_pairs=7,         # 极对数
        voltage=7.4           # 电源电压
    )
        # 初始化电机对象
    motor2 = Motor(
        pwm_pins=[26, 27, 14],  # A, B, C相引脚
        enable_pin=12,         # 使能引脚
        pole_pairs=7,         # 极对数
        voltage=7.4           # 电源电压
    )
    # 主循环
    while True:
        motor1.velocity_openloop(10) # 设置电机1目标速度为10rad/s（开环）
        motor2.velocity_openloop(-10) # 设置电机2目标速度为-10rad/s（开环）
        time.sleep_ms(6) # 循环间隔6ms
