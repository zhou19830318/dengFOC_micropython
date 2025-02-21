import math
import time
from machine import Pin, PWM, I2C, ADC
import ustruct
from AS5600 import Sensor_AS5600
from pid import PIDController
from lowpass_filter import LowPassFilter
from InlineCurrent import CurrSense

# 常量定义
_PI = 3.14159265359
_2PI = 6.28318530718
_PI_2 = 1.57079632679
_PI_3 = 1.0471975512
_3PI_2 = 4.71238898038
_SQRT3 = 1.73205080757
_SQRT3_2 = 0.86602540378
_1_SQRT3 = 0.57735026919
_2_SQRT3 = 1.15470053838

# 全局变量
voltage_power_supply = 12.6  # 默认电源电压

# 电机0引脚定义
M0_PP = 1
M0_DIR = 1
M0_pwmA = 32
M0_pwmB = 33
M0_pwmC = 25
enable0_pin = Pin(22, Pin.OUT)  # 使能引脚

# 电机1引脚定义
M1_PP = 1
M1_DIR = 1
M1_pwmA = 26
M1_pwmB = 27
M1_pwmC = 14
enable1_pin = Pin(12, Pin.OUT)  # 使能引脚

# 初始化PWM对象
pwm_channels = {
    'M0': {
        'A': PWM(Pin(M0_pwmA), freq=30000, duty=0),
        'B': PWM(Pin(M0_pwmB), freq=30000, duty=0),
        'C': PWM(Pin(M0_pwmC), freq=30000, duty=0)
    },
    'M1': {
        'A': PWM(Pin(M1_pwmA), freq=30000, duty=0),
        'B': PWM(Pin(M1_pwmB), freq=30000, duty=0),
        'C': PWM(Pin(M1_pwmC), freq=30000, duty=0)
    }
}

# 传感器初始化
S0 = Sensor_AS5600(0)
S1 = Sensor_AS5600(1)


# 滤波器初始化
M0_Vel_Flt = LowPassFilter(0.01)
M1_Vel_Flt = LowPassFilter(0.01)
M0_Curr_Flt = LowPassFilter(0.05)
M1_Curr_Flt = LowPassFilter(0.05)

# PID控制器初始化
vel_loop_M0 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000, limit=voltage_power_supply/2)
angle_loop_M0 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000, limit=100.0)
current_loop_M0 = PIDController(P=1.2, I=0.0, D=0.0, ramp=100000, limit=12.6)
vel_loop_M1 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000, limit=voltage_power_supply/2)
angle_loop_M1 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000, limit=100.0)
current_loop_M1 = PIDController(P=1.2, I=0.0, D=0.0, ramp=100000, limit=12.6)

# 电流传感器初始化
CS_M0 = CurrSense(0)
CS_M1 = CurrSense(1)

# 电机校准参数
S0_zero_electric_angle = 0.0
S1_zero_electric_angle = 0.0

# 工具函数
def _constrain(amt, low, high):
    return max(min(amt, high), low)

def _normalizeAngle(angle):
    a = math.fmod(angle, _2PI)
    return a if a >= 0 else a + _2PI

# PWM设置函数
def M0_setPwm(Ua, Ub, Uc):
    Ua = _constrain(Ua, 0.0, voltage_power_supply)
    Ub = _constrain(Ub, 0.0, voltage_power_supply)
    Uc = _constrain(Uc, 0.0, voltage_power_supply)
    
    dc_a = int(_constrain(Ua / voltage_power_supply, 0.0, 1.0) * 1023)
    dc_b = int(_constrain(Ub / voltage_power_supply, 0.0, 1.0) * 1023)
    dc_c = int(_constrain(Uc / voltage_power_supply, 0.0, 1.0) * 1023)
    
    pwm_channels['M0']['A'].duty(dc_a)
    pwm_channels['M0']['B'].duty(dc_b)
    pwm_channels['M0']['C'].duty(dc_c)

def M1_setPwm(Ua, Ub, Uc):
    Ua = _constrain(Ua, 0.0, voltage_power_supply)
    Ub = _constrain(Ub, 0.0, voltage_power_supply)
    Uc = _constrain(Uc, 0.0, voltage_power_supply)
    
    dc_a = int(_constrain(Ua / voltage_power_supply, 0.0, 1.0) * 1023)
    dc_b = int(_constrain(Ub / voltage_power_supply, 0.0, 1.0) * 1023)
    dc_c = int(_constrain(Uc / voltage_power_supply, 0.0, 1.0) * 1023)
    
    pwm_channels['M1']['A'].duty(dc_a)
    pwm_channels['M1']['B'].duty(dc_b)
    pwm_channels['M1']['C'].duty(dc_c)

# 扭矩设置函数
def M0_setTorque(Uq, angle_el):
    global voltage_power_supply
    if Uq < 0:
        angle_el += _PI
    Uq = abs(Uq)
    
    angle_el = _normalizeAngle(angle_el + _PI_2)
    sector = int(math.floor(angle_el / _PI_3)) + 1
    
    T1 = _SQRT3 * math.sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply
    T2 = _SQRT3 * math.sin(angle_el - (sector-1.0)*_PI_3) * Uq / voltage_power_supply
    T0 = 1 - T1 - T2
    
    sector_map = {
        1: (T1+T2+T0/2, T2+T0/2, T0/2),
        2: (T1+T0/2, T1+T2+T0/2, T0/2),
        3: (T0/2, T1+T2+T0/2, T2+T0/2),
        4: (T0/2, T1+T0/2, T1+T2+T0/2),
        5: (T2+T0/2, T0/2, T1+T2+T0/2),
        6: (T1+T2+T0/2, T0/2, T1+T0/2)
    }
    
    Ta, Tb, Tc = sector_map.get(sector, (0, 0, 0))
    
    Ua = Ta * voltage_power_supply
    Ub = Tb * voltage_power_supply
    Uc = Tc * voltage_power_supply
    M0_setPwm(Ua, Ub, Uc)

def M1_setTorque(Uq, angle_el):
    global voltage_power_supply
    if Uq < 0:
        angle_el += _PI
    Uq = abs(Uq)
    
    angle_el = _normalizeAngle(angle_el + _PI_2)
    sector = int(math.floor(angle_el / _PI_3)) + 1
    
    T1 = _SQRT3 * math.sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply
    T2 = _SQRT3 * math.sin(angle_el - (sector-1.0)*_PI_3) * Uq / voltage_power_supply
    T0 = 1 - T1 - T2
    
    sector_map = {
        1: (T1+T2+T0/2, T2+T0/2, T0/2),
        2: (T1+T0/2, T1+T2+T0/2, T0/2),
        3: (T0/2, T1+T2+T0/2, T2+T0/2),
        4: (T0/2, T1+T0/2, T1+T2+T0/2),
        5: (T2+T0/2, T0/2, T1+T2+T0/2),
        6: (T1+T2+T0/2, T0/2, T1+T0/2)
    }
    
    Ta, Tb, Tc = sector_map.get(sector, (0, 0, 0))
    
    Ua = Ta * voltage_power_supply
    Ub = Tb * voltage_power_supply
    Uc = Tc * voltage_power_supply
    M1_setPwm(Ua, Ub, Uc)

# 系统控制函数
def DFOC_enable():
    enable0_pin.value(1)
    enable1_pin.value(1)

def DFOC_disable():
    enable0_pin.value(0)
    enable1_pin.value(1)

def DFOC_Vbus(power_supply):
    global voltage_power_supply
    voltage_power_supply = power_supply
    # 初始化电流传感器
    CS_M0.init()
    CS_M1.init()
    # 初始化编码器
    S0.Sensor_init(0, 18, 19)
    S1.Sensor_init(1, 5, 23)
    print("System initialization completed")

def S0_electricalAngle():
    return _normalizeAngle(M0_DIR * M0_PP * S0.getMechanicalAngle() - S0_zero_electric_angle)

def S1_electricalAngle():
    return _normalizeAngle(M1_DIR * M1_PP * S1.getMechanicalAngle() - S1_zero_electric_angle)

# 传感器校准函数
def DFOC_M0_alignSensor(PP, DIR):
    global M0_PP, M0_DIR, S0_zero_electric_angle
    M0_PP = PP
    M0_DIR = DIR
    M0_setTorque(3.0, _3PI_2)
    time.sleep(1)
    S0.Sensor_update()
    S0_zero_electric_angle = S0_electricalAngle()
    M0_setTorque(0.0, _3PI_2)
    print("M0 zero electrical angle:", S0_zero_electric_angle)

def DFOC_M1_alignSensor(PP, DIR):
    global M1_PP, M1_DIR, S1_zero_electric_angle
    M1_PP = PP
    M1_DIR = DIR
    M1_setTorque(3.0, _3PI_2)
    time.sleep(1)
    S1.Sensor_update()
    S1_zero_electric_angle = S1_electricalAngle()
    M1_setTorque(0.0, _3PI_2)
    print("M0 zero electrical angle:", S1_zero_electric_angle)

def DFOC_M0_Angle():
    return M0_DIR * S0.getAngle()

def DFOC_M1_Angle():
    return M1_DIR * S1.getAngle()

def cal_Iq_Id(current_a, current_b, angle_el):
    I_alpha = current_a
    I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b
    ct = math.cos(angle_el)
    st = math.sin(angle_el)
    return I_beta * ct - I_alpha * st  # I_q

def DFOC_M0_Current():
    I_q = cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, S0_electricalAngle())
    return M0_Curr_Flt(I_q)

def DFOC_M1_Current():
    I_q = cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, S1_electricalAngle())
    return M1_Curr_Flt(I_q)

def DFOC_M0_Velocity():
    vel = M0_DIR * S0.getVelocity()
    return M0_Vel_Flt(vel)

def DFOC_M1_Velocity():
    vel = M1_DIR * S1.getVelocity()
    return M1_Vel_Flt(vel)

# 电流环控制
def DFOC_M0_setTorque(target):
    current_error = target - DFOC_M0_Current()
    Uq = current_loop_M0(current_error)
    M0_setTorque(Uq, S0_electricalAngle())

def DFOC_M1_setTorque(target):
    current_error = target - DFOC_M1_Current()
    Uq = current_loop_M1(current_error)
    M1_setTorque(Uq, S1_electricalAngle())

def DFOC_M0_set_Velocity_Angle(Target):  #//角度-速度-力 位置闭环
    DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI) - DFOC_M0_Velocity()))  #//改进后

def DFOC_M1_set_Velocity_Angle(Target):  #//角度-速度-力 位置闭环
    DFOC_M1_setTorque(DFOC_M1_VEL_PID(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI) - DFOC_M1_Velocity()))  #//改进后


# 速度环控制
def DFOC_M0_setVelocity(target):
    velocity_error = (target - DFOC_M0_Velocity()) * 180 / _PI
    torque_target = vel_loop_M0(velocity_error)
    DFOC_M0_setTorque(torque_target)

def DFOC_M1_setVelocity(target):
    velocity_error = (target - DFOC_M1_Velocity()) * 180 / _PI
    torque_target = vel_loop_M1(velocity_error)
    DFOC_M1_setTorque(torque_target)
    
def DFOC_M0_set_Force_Angle(Target):  #//力位闭环
    DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI))  #//改进后

def DFOC_M1_set_Force_Angle(Target):  #//力位闭环
    DFOC_M1_setTorque(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI))  #//改进后

# FOC主循环
def runFOC():
    S0.Sensor_update()
    S1.Sensor_update()
    CS_M0.getPhaseCurrents()
    CS_M1.getPhaseCurrents()
    
'''
# 示例使用
if __name__ == '__main__':
    DFOC_Vbus(12.6)
    DFOC_enable()
    DFOC_M0_alignSensor(7, 1)
    
    while True:
        runFOC()
        DFOC_M0_setVelocity(1)  # 目标速度10rad/s
        time.sleep_ms(1)
'''
