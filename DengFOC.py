import machine
import utime
import math

# 从其他重构的文件中导入类
from as5600 import Sensor_AS5600 # Assuming as5600.py is in the same directory or sys.path
from lowpass_filter import LowPassFilter
from pid import PIDController
from inline_current import CurrSense

# --- Constants --- (mirrored from C++ defines and globals)
_PI = math.pi
_2PI = 2 * _PI
_PI_2 = _PI / 2.0
_PI_3 = _PI / 3.0
_3PI_2 = 3.0 * _PI / 2.0

_SQRT3 = math.sqrt(3.0)
_SQRT3_2 = math.sqrt(3.0) / 2.0
_1_SQRT3 = 1.0 / math.sqrt(3.0)
_2_SQRT3 = 2.0 / math.sqrt(3.0)

# --- Global Variables --- (equivalent to C++ global variables)
voltage_power_supply = 12.0  # Default, should be set by DFOC_Vbus

# Motor 0 specific parameters
S0_zero_electric_angle = 0.0
M0_PP = 7  # Default pole pairs, should be set by alignSensor
M0_DIR = 1 # Default direction, should be set by alignSensor
M0_pwmA_pin_num = 32
M0_pwmB_pin_num = 33
M0_pwmC_pin_num = 25

# Motor 1 specific parameters
S1_zero_electric_angle = 0.0
M1_PP = 7  # Default pole pairs
M1_DIR = 1 # Default direction
M1_pwmA_pin_num = 26
M1_pwmB_pin_num = 27
M1_pwmC_pin_num = 14

enable_pin_num = 12
enable_pin = None # machine.Pin object

# PWM objects (will be machine.PWM instances)
M0_pwmA, M0_pwmB, M0_pwmC = None, None, None
M1_pwmA, M1_pwmB, M1_pwmC = None, None, None
PWM_FREQUENCY = 30000 # 30 kHz
PWM_DUTY_RESOLUTION = 8 # 8-bit resolution (0-255 for duty cycle)
# For machine.PWM, duty_u16 uses 0-65535. We'll need to scale.
PWM_MAX_DUTY = (1 << PWM_DUTY_RESOLUTION) -1 # 255 for 8-bit
PWM_MAX_DUTY_U16 = 65535 # For duty_u16

# Low Pass Filters
M0_Vel_Flt = LowPassFilter(0.01)  # Tf = 10ms for M0 velocity
M1_Vel_Flt = LowPassFilter(0.01)  # Tf = 10ms for M1 velocity
M0_Curr_Flt = LowPassFilter(0.05) # Tf = 5ms for M0 current (Iq)
M1_Curr_Flt = LowPassFilter(0.05) # Tf = 5ms for M1 current (Iq)

# PID Controllers - Initialized with default values from C++
# These will be updated by DFOC_M0_SET_..._PID functions
# Note: limit for vel_loop is voltage_power_supply / 2, which needs voltage_power_supply to be set first.
vel_loop_M0 = None
angle_loop_M0 = PIDController(P=20.0, I=0.0, D=0.0, ramp=100000.0, limit=100.0) # Example values, limit for angle typically in rad/s or similar
current_loop_M0 = PIDController(P=1.2, I=0.0, D=0.0, ramp=100000.0, limit=12.6) # Limit for current loop is often Uq max

vel_loop_M1 = None
angle_loop_M1 = PIDController(P=20.0, I=0.0, D=0.0, ramp=100000.0, limit=100.0)
current_loop_M1 = PIDController(P=1.2, I=0.0, D=0.0, ramp=100000.0, limit=12.6)

# AS5600 Sensors
# I2C pins from C++ DengFOC.cpp DFOC_Vbus:
# S0_I2C.begin(19, 18, 400000UL); SCL=19, SDA=18 for M0
# S1_I2C.begin(23, 5, 400000UL);  SCL=23, SDA=5  for M1
S0 = Sensor_AS5600(Mot_Num=0, i2c_id=0, scl_pin=19, sda_pin=18)
S1 = Sensor_AS5600(Mot_Num=1, i2c_id=1, scl_pin=23, sda_pin=5) # Assuming I2C bus 1 for M1

# Current Sensors
CS_M0 = CurrSense(Mot_Num=0)
CS_M1 = CurrSense(Mot_Num=1)

# Serial communication variables (simplified for MicroPython)
M0_target_serial = 0.0
M1_target_serial = 0.0

# --- Helper Functions --- 
def _constrain(amt, low, high):
    return max(low, min(amt, high))

def _normalizeAngle(angle):
    a = math.fmod(angle, _2PI)
    return a if a >= 0 else (a + _2PI)

# --- PWM Functions --- 
def _set_pwm_duty(pwm_obj, duty_float_0_to_1):
    # duty_float is 0.0 to 1.0
    # machine.PWM.duty_u16 expects 0 to 65535
    duty_u16_val = int(_constrain(duty_float_0_to_1 * PWM_MAX_DUTY_U16, 0, PWM_MAX_DUTY_U16))
    if pwm_obj:
        pwm_obj.duty_u16(duty_u16_val)

def M0_setPwm(Ua, Ub, Uc):
    global voltage_power_supply
    Ua = _constrain(Ua, 0.0, voltage_power_supply)
    Ub = _constrain(Ub, 0.0, voltage_power_supply)
    Uc = _constrain(Uc, 0.0, voltage_power_supply)

    dc_a = _constrain(Ua / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0
    dc_b = _constrain(Ub / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0
    dc_c = _constrain(Uc / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0
    
    _set_pwm_duty(M0_pwmA, dc_a)
    _set_pwm_duty(M0_pwmB, dc_b)
    _set_pwm_duty(M0_pwmC, dc_c)

def M1_setPwm(Ua, Ub, Uc):
    global voltage_power_supply
    Ua = _constrain(Ua, 0.0, voltage_power_supply)
    Ub = _constrain(Ub, 0.0, voltage_power_supply)
    Uc = _constrain(Uc, 0.0, voltage_power_supply)

    dc_a = _constrain(Ua / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0
    dc_b = _constrain(Ub / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0
    dc_c = _constrain(Uc / voltage_power_supply, 0.0, 1.0) if voltage_power_supply else 0.0

    _set_pwm_duty(M1_pwmA, dc_a)
    _set_pwm_duty(M1_pwmB, dc_b)
    _set_pwm_duty(M1_pwmC, dc_c)

# --- Torque Setting (SVPWM) --- 
def _set_torque_svpwm(Uq, angle_el, motor_set_pwm_func, v_bus):
    if v_bus <= 0: # Cannot operate without bus voltage
        motor_set_pwm_func(0,0,0)
        return

    # Normalize Uq to be a fraction of Vbus/sqrt(3) for modulation index, or handle Uq directly as voltage
    # The original C++ code constrains Uq to +/- v_bus/2. Here Uq is target voltage.
    # Uq = _constrain(Uq, -v_bus / 2.0, v_bus / 2.0) # This was for a different setTorque version
    # For SVPWM, Uq is the magnitude of the voltage vector. Max Uq is Vdc / sqrt(3) for linear region.
    # The C++ code seems to take Uq as the target voltage for the q-axis.
    # Let's assume Uq is the magnitude of the space vector |Vs|
    
    # The C++ code's SVPWM implementation:
    # if (Uq < 0) angle_el += _PI; Uq = abs(Uq);
    # angle_el = _normalizeAngle(angle_el + _PI_2);
    # ... sector calculation ... T1, T2, T0 ... Ta, Tb, Tc
    # This is a common SVPWM implementation structure.

    # Scale Uq to modulation index m_a = Uq / (Vdc / sqrt(3)) if Uq is phase peak.
    # Or, if Uq is the magnitude of the reference vector |Vref|, then max |Vref| = Vdc/sqrt(3) for linear modulation.
    # The C++ code uses Uq directly in T1, T2 calculation scaled by 1/voltage_power_supply.
    # This implies Uq is already a voltage value.
    # Max Uq for linear SVPWM is Vdc/sqrt(3). If Uq can be Vdc/2, it might enter overmodulation.
    # Let's cap Uq to Vdc/sqrt(3) to stay in linear region for SVPWM, or trust original scaling.
    # The original code constrains Uq in current_loop.limit, e.g., 12.6V. If Vbus is 12V, this is high.
    # Let's assume Uq is the desired q-axis voltage.
    # The SVPWM part takes Uq as the magnitude of the voltage vector to synthesize.

    Uq_abs = abs(Uq)
    if Uq < 0:
        angle_el_eff = _normalizeAngle(angle_el + _PI)
    else:
        angle_el_eff = _normalizeAngle(angle_el)
    
    # angle_el_eff is the angle of the stator voltage vector Vs.
    # The SVPWM algorithm typically takes the angle of the reference vector and its magnitude.
    # The C++ code shifts angle by PI/2: angle_el_svpwm = _normalizeAngle(angle_el_eff + _PI_2)
    # This shift is standard in some FOC implementations where angle_el is rotor flux angle.
    # The voltage vector should lead the flux vector by 90 degrees for pure torque in PMSM (Ud=0).
    angle_svpwm = _normalizeAngle(angle_el_eff + _PI_2)

    sector = math.floor(angle_svpwm / _PI_3) + 1

    # Duty cycle calculations T1, T2, T0 (relative to switching period Tpwm)
    # Uq_abs is |Vs|. Max |Vs| for linear modulation is Vdc / sqrt(3).
    # If Uq_abs is higher, overmodulation occurs. Let's cap it for stability.
    # Uq_abs_constrained = _constrain(Uq_abs, 0, v_bus / _SQRT3) # Cap to linear region
    Uq_abs_constrained = _constrain(Uq_abs, 0, v_bus * 0.95) # Allow some overmodulation, or use v_bus/2 as in original torque limit
                                                            # The original current_loop.limit is 12.6, if v_bus is 12, this is > v_bus/2
                                                            # Let's use a limit related to v_bus, e.g. v_bus * 0.577 or v_bus * 0.707
                                                            # For now, let's assume Uq is appropriately limited by PID.

    # Original C++: T1 = _SQRT3 * sin(sector * _PI_3 - angle_svpwm) * Uq_abs_constrained / v_bus;
    # This is effectively: T1 = ( |Vs| / (2/3 * Vdc) ) * (2/sqrt(3)) * sin(sector*pi/3 - angle_svpwm)
    # Or more standard: T1 = (sqrt(3) * |Vs| / Vdc) * sin(sector*pi/3 - angle_svpwm)
    # Let m = |Vs| / (Vdc / sqrt(3)) be modulation index. Max m=1 for linear.
    # T1 = m * sin(sector*pi/3 - angle_svpwm)
    # T2 = m * sin(angle_svpwm - (sector-1)*pi/3)
    # The C++ code's Uq_abs / v_bus is like |Vs|/Vdc. So factor sqrt(3) is needed.

    # Recalculating T1, T2 based on standard SVPWM formulas where times are fractions of PWM period:
    # X = (|Vs| * sqrt(3) / Vdc) * sin(angle_svpwm - (sector-1)*pi/3)  == T2 in some notations
    # Y = (|Vs| * sqrt(3) / Vdc) * sin(sector*pi/3 - angle_svpwm)      == T1 in some notations
    # Let's match the C++ variable names T1, T2.
    # Original C++: float T1 = _SQRT3 * sin(sector * _PI_3 - angle_svpwm) * Uq_abs / v_bus;
    # Original C++: float T2 = _SQRT3 * sin(angle_svpwm - (sector - 1.0) * _PI_3) * Uq_abs / v_bus;
    # These seem to be the time durations for the two active vectors, scaled by sqrt(3) * Uq_abs / v_bus.
    # This scaling factor, if Uq_abs is |Vs|, should be |Vs| * sqrt(3) / Vdc for T1, T2 to be < 1.
    # If Uq_abs is already scaled (e.g. Uq_abs = |Vs| * sqrt(3) / Vdc), then T1, T2 are direct sines.
    # Given Uq is an output of PID limited by v_bus/2 or similar, it's a voltage.
    # So, Uq_abs / v_bus is a ratio. Let's assume the formulas are correct as is.
    
    # Cap Uq_abs to prevent T1+T2 > 1, which would make T0 negative.
    # Max value for (sin(A) + sin(B)) where A+B = pi/3 is 1. So sqrt(3)*Uq_abs/v_bus * 1 <= 1
    # So, Uq_abs <= v_bus / sqrt(3).
    Uq_for_T_calc = _constrain(Uq_abs, 0, v_bus / _SQRT3) # Ensure linear modulation for T0, T1, T2 sum to 1

    val1 = sector * _PI_3 - angle_svpwm
    val2 = angle_svpwm - (sector - 1.0) * _PI_3

    T1 = _SQRT3 * math.sin(val1) * Uq_for_T_calc / v_bus
    T2 = _SQRT3 * math.sin(val2) * Uq_for_T_calc / v_bus

    # Ensure T1 and T2 are not negative due to float precision with sin near 0
    T1 = max(0, T1)
    T2 = max(0, T2)

    if (T1 + T2 > 1.0):
        # Overmodulation, scale T1 and T2 to sum to 1
        T_sum = T1 + T2
        T1 = T1 / T_sum
        T2 = T2 / T_sum
        T0 = 0.0
    else:
        T0 = 1.0 - T1 - T2

    Ta, Tb, Tc = 0,0,0 # Duty cycles for phases A, B, C
    if sector == 1:
        Ta = T1 + T2 + T0 / 2.0
        Tb = T2 + T0 / 2.0
        Tc = T0 / 2.0
    elif sector == 2:
        Ta = T1 + T0 / 2.0
        Tb = T1 + T2 + T0 / 2.0
        Tc = T0 / 2.0
    elif sector == 3:
        Ta = T0 / 2.0
        Tb = T1 + T2 + T0 / 2.0
        Tc = T2 + T0 / 2.0
    elif sector == 4:
        Ta = T0 / 2.0
        Tb = T1 + T0 / 2.0
        Tc = T1 + T2 + T0 / 2.0
    elif sector == 5:
        Ta = T2 + T0 / 2.0
        Tb = T0 / 2.0
        Tc = T1 + T2 + T0 / 2.0
    elif sector == 6:
        Ta = T1 + T2 + T0 / 2.0
        Tb = T0 / 2.0
        Tc = T1 + T0 / 2.0
    # Default case (should not happen if angle_svpwm is normalized)
    # else: Ta, Tb, Tc remain 0,0,0

    # Resulting Ta, Tb, Tc are duty cycles (0 to 1). Convert to phase voltages.
    Ua = Ta * v_bus
    Ub = Tb * v_bus
    Uc = Tc * v_bus
    motor_set_pwm_func(Ua, Ub, Uc)

def M0_setTorque(Uq, angle_el):
    _set_torque_svpwm(Uq, angle_el, M0_setPwm, voltage_power_supply)

def M1_setTorque(Uq, angle_el):
    _set_torque_svpwm(Uq, angle_el, M1_setPwm, voltage_power_supply)

# --- Enable/Disable --- 
def DFOC_enable():
    if enable_pin: enable_pin.on() # Or .value(1)
    print("Motors enabled (simulated if enable_pin not configured)")

def DFOC_disable():
    if enable_pin: enable_pin.off() # Or .value(0)
    # Also set PWMs to 0 for safety
    M0_setPwm(0,0,0)
    M1_setPwm(0,0,0)
    print("Motors disabled (simulated if enable_pin not configured)")

# --- Initialization --- 
def DFOC_Vbus(power_supply):
    global voltage_power_supply, enable_pin
    global M0_pwmA, M0_pwmB, M0_pwmC, M1_pwmA, M1_pwmB, M1_pwmC
    global vel_loop_M0, vel_loop_M1

    voltage_power_supply = power_supply
    print(f"Bus voltage set to: {voltage_power_supply}V")

    # Initialize PID controllers that depend on voltage_power_supply
    vel_loop_M0 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000.0, limit=voltage_power_supply / 2.0)
    vel_loop_M1 = PIDController(P=2.0, I=0.0, D=0.0, ramp=100000.0, limit=voltage_power_supply / 2.0)

    # Configure enable pin
    try:
        enable_pin = machine.Pin(enable_pin_num, machine.Pin.OUT)
        DFOC_disable() # Start with motors disabled
    except Exception as e:
        print(f"Warning: Could not initialize enable pin {enable_pin_num}: {e}")

    # Configure PWM pins
    try:
        M0_pwmA = machine.PWM(machine.Pin(M0_pwmA_pin_num), freq=PWM_FREQUENCY)
        M0_pwmB = machine.PWM(machine.Pin(M0_pwmB_pin_num), freq=PWM_FREQUENCY)
        M0_pwmC = machine.PWM(machine.Pin(M0_pwmC_pin_num), freq=PWM_FREQUENCY)
        M1_pwmA = machine.PWM(machine.Pin(M1_pwmA_pin_num), freq=PWM_FREQUENCY)
        M1_pwmB = machine.PWM(machine.Pin(M1_pwmB_pin_num), freq=PWM_FREQUENCY)
        M1_pwmC = machine.PWM(machine.Pin(M1_pwmC_pin_num), freq=PWM_FREQUENCY)
        print("PWM initialized.")
    except Exception as e:
        print(f"Error initializing PWM: {e}. PWM functionality will be simulated.")
        # Create dummy PWM objects if real ones fail, for simulation
        class DummyPWM: 
            def duty_u16(self, val): pass
            def deinit(self): pass
        M0_pwmA, M0_pwmB, M0_pwmC = DummyPWM(), DummyPWM(), DummyPWM()
        M1_pwmA, M1_pwmB, M1_pwmC = DummyPWM(), DummyPWM(), DummyPWM()

    # Initialize Sensors
    try:
        S0.sensor_init() # Uses I2C pins defined in S0 constructor
        S1.sensor_init() # Uses I2C pins defined in S1 constructor
        print("AS5600 Sensors initialized.")
    except Exception as e:
        print(f"Error initializing AS5600 sensors: {e}")

    # Initialize Current Sensors
    try:
        CS_M0.init()
        CS_M1.init()
        print("Current Sensors initialized.")
    except Exception as e:
        print(f"Error initializing Current Sensors: {e}")
    
    print("DFOC_Vbus setup complete.")

# --- Electrical Angle --- 
def S0_electricalAngle():
    # Ensure S0.getMechanicalAngle() is up-to-date via S0.sensor_update() if needed frequently
    # sensor_update() is called in runFOC()
    mechanical_angle = S0.get_mechanical_angle()
    return _normalizeAngle(float(M0_DIR * M0_PP) * mechanical_angle - S0_zero_electric_angle)

def S1_electricalAngle():
    mechanical_angle = S1.get_mechanical_angle()
    return _normalizeAngle(float(M1_DIR * M1_PP) * mechanical_angle - S1_zero_electric_angle)

# --- Sensor Alignment --- 
def DFOC_M0_alignSensor(PP, DIR):
    global M0_PP, M0_DIR, S0_zero_electric_angle
    M0_PP = PP
    M0_DIR = DIR
    print(f"Aligning M0: PP={M0_PP}, DIR={M0_DIR}")
    if not voltage_power_supply > 0:
        print("Error: Bus voltage not set. Cannot align sensor.")
        return
    
    DFOC_enable() # Enable driver
    # Apply a voltage to align the rotor, e.g., 3V as in C++
    # This voltage should be enough to move the motor but not too high.
    # The angle _3PI_2 corresponds to aligning d-axis with alpha-axis (Ualpha=Vd, Ubeta=0)
    # Or q-axis with beta-axis (Ualpha=0, Ubeta=Vq)
    # M0_setTorque(Uq, angle_el) -> angle_el is electrical angle of rotor d-axis.
    # We want to command a DC field. For PMSM, often align to q-axis or d-axis.
    # Original C++ uses M0_setTorque(3, _3PI_2). This means Uq=3V, electrical_angle = 270 degrees.
    # This will try to drive the q-axis of the rotor to 270 deg electrical.
    print("Applying alignment voltage to M0...")
    M0_setTorque(3.0, _3PI_2) # Apply 3V to q-axis at electrical angle 270 deg
    utime.sleep_ms(1000) # Wait for rotor to settle
    
    S0.sensor_update() # Update sensor reading
    # The electrical angle calculated now IS the offset if rotor d-axis is aligned with reference d-axis.
    # The C++ code calculates S0_electricalAngle() which is (DIR*PP*mech_angle - zero_el_angle).
    # At this point, zero_el_angle is 0. So it reads DIR*PP*mech_angle.
    # This becomes the new S0_zero_electric_angle.
    S0_zero_electric_angle = _normalizeAngle(float(M0_DIR * M0_PP) * S0.get_mechanical_angle())
    
    M0_setTorque(0, _3PI_2) # Release torque
    DFOC_disable()
    print(f"M0 alignment complete. Zero electric angle: {S0_zero_electric_angle:.4f} rad")

def DFOC_M1_alignSensor(PP, DIR):
    global M1_PP, M1_DIR, S1_zero_electric_angle
    M1_PP = PP
    M1_DIR = DIR
    print(f"Aligning M1: PP={M1_PP}, DIR={M1_DIR}")
    if not voltage_power_supply > 0: return
    DFOC_enable()
    M1_setTorque(3.0, _3PI_2)
    utime.sleep_ms(1000)
    S1.sensor_update()
    S1_zero_electric_angle = _normalizeAngle(float(M1_DIR * M1_PP) * S1.get_mechanical_angle())
    M1_setTorque(0, _3PI_2)
    DFOC_disable()
    print(f"M1 alignment complete. Zero electric angle: {S1_zero_electric_angle:.4f} rad")

# --- Sensor Reading Wrappers --- 
def DFOC_M0_Angle(): return M0_DIR * S0.get_angle()
def DFOC_M1_Angle(): return M1_DIR * S1.get_angle()

def DFOC_M0_Velocity():
    vel_M0_ori = S0.get_velocity()
    return M0_Vel_Flt(M0_DIR * vel_M0_ori)

def DFOC_M1_Velocity():
    vel_M1_ori = S1.get_velocity()
    return M1_Vel_Flt(M1_DIR * vel_M1_ori)

# --- Current Calculation (Clarke/Park) --- 
def cal_Iq_Id(current_a, current_b, angle_el):
    # Clarke transform (assuming balanced currents, Ia+Ib+Ic=0, so Ic can be derived if needed)
    # Standard Clarke: Ialpha = Ia; Ibeta = (Ia + 2*Ib) / sqrt(3)
    # The C++ uses: I_alpha = current_a; I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    # This is equivalent to: I_beta = (current_a + 2*current_b) / sqrt(3)
    I_alpha = current_a
    I_beta = (_1_SQRT3 * current_a) + (_2_SQRT3 * current_b)

    # Park transform
    ct = math.cos(angle_el)
    st = math.sin(angle_el)
    # I_d = I_alpha * ct + I_beta * st # Id not used in this function in C++
    I_q = I_beta * ct - I_alpha * st
    return I_q # Only Iq is returned by C++ version

def DFOC_M0_Current():
    # CS_M0.current_a, CS_M0.current_b should be updated by runFOC -> CS_M0.getPhaseCurrents()
    I_q_M0_ori = cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, S0_electricalAngle())
    return M0_Curr_Flt(I_q_M0_ori)

def DFOC_M1_Current():
    I_q_M1_ori = cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, S1_electricalAngle())
    return M1_Curr_Flt(I_q_M1_ori)

# --- PID Setters --- (Assumes PID objects are already created)
def DFOC_M0_SET_ANGLE_PID(P, I, D, ramp, limit):
    global angle_loop_M0
    angle_loop_M0 = PIDController(P,I,D,ramp,limit)
    print(f"M0 Angle PID set: P={P}, I={I}, D={D}, Ramp={ramp}, Limit={limit}")

def DFOC_M0_SET_VEL_PID(P, I, D, ramp, limit):
    global vel_loop_M0
    vel_loop_M0 = PIDController(P,I,D,ramp,limit)
    print(f"M0 Velocity PID set: P={P}, I={I}, D={D}, Ramp={ramp}, Limit={limit}")

def DFOC_M0_SET_CURRENT_PID(P, I, D, ramp, limit): # Original C++ had no limit here, but PID class needs it
    global current_loop_M0
    current_loop_M0 = PIDController(P,I,D,ramp,limit)
    print(f"M0 Current PID set: P={P}, I={I}, D={D}, Ramp={ramp}, Limit={limit}")

# ... Similar setters for M1 ...
def DFOC_M1_SET_ANGLE_PID(P, I, D, ramp, limit):
    global angle_loop_M1
    angle_loop_M1 = PIDController(P,I,D,ramp,limit)

def DFOC_M1_SET_VEL_PID(P, I, D, ramp, limit):
    global vel_loop_M1
    vel_loop_M1 = PIDController(P,I,D,ramp,limit)

def DFOC_M1_SET_CURRENT_PID(P, I, D, ramp, limit):
    global current_loop_M1
    current_loop_M1 = PIDController(P,I,D,ramp,limit)

# --- PID Getters (Run PID loop) --- 
def DFOC_M0_ANGLE_PID(error): return angle_loop_M0(error) if angle_loop_M0 else 0
def DFOC_M0_VEL_PID(error): return vel_loop_M0(error) if vel_loop_M0 else 0
# Current PID is usually called internally by DFOC_M0_setTorque, not exposed like this.
# def DFOC_M0_CURRENT_PID(error): return current_loop_M0(error)

def DFOC_M1_ANGLE_PID(error): return angle_loop_M1(error) if angle_loop_M1 else 0
def DFOC_M1_VEL_PID(error): return vel_loop_M1(error) if vel_loop_M1 else 0

# --- High-Level Control Modes --- 
# Note: Original C++ uses (Target - DFOC_M0_Angle()) * 180 / PI.
# This converts radians to degrees for the PID. If PIDs are tuned for radians, remove *180/PI.
# Assuming PIDs are tuned for inputs in radians/rad/s for angle/velocity.
# If PID expects degrees, keep the conversion.
# For simplicity, let's assume PID inputs are in SI units (radians, rad/s).
CONV_FACTOR = 1.0 # Set to 180/_PI if PIDs expect degrees/deg/s

def DFOC_M0_setTorque_CurrentClosedLoop(Target_current):
    # Target_current is desired Iq.
    # PID output is Uq.
    if not current_loop_M0: print("M0 Current PID not initialized!"); return
    error_current = Target_current - DFOC_M0_Current()
    Uq_command = current_loop_M0(error_current)
    M0_setTorque(Uq_command, S0_electricalAngle())

def DFOC_M1_setTorque_CurrentClosedLoop(Target_current):
    if not current_loop_M1: print("M1 Current PID not initialized!"); return
    error_current = Target_current - DFOC_M1_Current()
    Uq_command = current_loop_M1(error_current)
    M1_setTorque(Uq_command, S1_electricalAngle())

def DFOC_M0_set_Velocity_Angle(Target_angle_rad):
    # Cascade: Angle_PID -> Velocity_PID -> Current_PID (via setTorque_CurrentClosedLoop)
    if not angle_loop_M0 or not vel_loop_M0: print("M0 Angle/Vel PID not initialized!"); return
    error_angle = (Target_angle_rad - DFOC_M0_Angle()) * CONV_FACTOR
    target_velocity_rad_s = angle_loop_M0(error_angle)
    
    error_velocity = (target_velocity_rad_s - DFOC_M0_Velocity()) * CONV_FACTOR
    target_current_Iq = vel_loop_M0(error_velocity)
    DFOC_M0_setTorque_CurrentClosedLoop(target_current_Iq)

def DFOC_M1_set_Velocity_Angle(Target_angle_rad):
    if not angle_loop_M1 or not vel_loop_M1: print("M1 Angle/Vel PID not initialized!"); return
    error_angle = (Target_angle_rad - DFOC_M1_Angle()) * CONV_FACTOR
    target_velocity_rad_s = angle_loop_M1(error_angle)
    
    error_velocity = (target_velocity_rad_s - DFOC_M1_Velocity()) * CONV_FACTOR
    target_current_Iq = vel_loop_M1(error_velocity)
    DFOC_M1_setTorque_CurrentClosedLoop(target_current_Iq)

def DFOC_M0_setVelocity(Target_velocity_rad_s):
    # Cascade: Velocity_PID -> Current_PID
    if not vel_loop_M0: print("M0 Velocity PID not initialized!"); return
    error_velocity = (Target_velocity_rad_s - DFOC_M0_Velocity()) * CONV_FACTOR
    target_current_Iq = vel_loop_M0(error_velocity)
    DFOC_M0_setTorque_CurrentClosedLoop(target_current_Iq)

def DFOC_M1_setVelocity(Target_velocity_rad_s):
    if not vel_loop_M1: print("M1 Velocity PID not initialized!"); return
    error_velocity = (Target_velocity_rad_s - DFOC_M1_Velocity()) * CONV_FACTOR
    target_current_Iq = vel_loop_M1(error_velocity)
    DFOC_M1_setTorque_CurrentClosedLoop(target_current_Iq)

def DFOC_M0_set_Force_Angle(Target_angle_rad): # Force implies torque control via angle error
    # Cascade: Angle_PID -> Current_PID
    if not angle_loop_M0: print("M0 Angle PID not initialized!"); return
    error_angle = (Target_angle_rad - DFOC_M0_Angle()) * CONV_FACTOR
    target_current_Iq = angle_loop_M0(error_angle) # Output of angle PID is target torque/current
    DFOC_M0_setTorque_CurrentClosedLoop(target_current_Iq)

def DFOC_M1_set_Force_Angle(Target_angle_rad):
    if not angle_loop_M1: print("M1 Angle PID not initialized!"); return
    error_angle = (Target_angle_rad - DFOC_M1_Angle()) * CONV_FACTOR
    target_current_Iq = angle_loop_M1(error_angle)
    DFOC_M1_setTorque_CurrentClosedLoop(target_current_Iq)

# --- Main Loop Function --- 
def runFOC():
    S0.sensor_update()
    S1.sensor_update()
    CS_M0.get_phase_currents()
    CS_M1.get_phase_currents()
    # Other periodic tasks could be added here if necessary

# --- Serial Communication (Simplified) --- 
# MicroPython's serial handling is different. This is a placeholder.
# For ESP32, you might use machine.UART or sys.stdin for input.
_received_chars = ""
def serialReceiveUserCommand():
    global _received_chars, M0_target_serial, M1_target_serial # Assuming M0_target_serial is used
    # This is a very basic example. Robust serial handling is more complex.
    # e.g. using select.select for non-blocking read from sys.stdin
    # Or machine.UART.any() and machine.UART.read()
    # For simplicity, let's assume a global variable might be set by another part of the code
    # or a more sophisticated input mechanism.
    # The original C++ code parses a string like "target_val\n"
    # command = "" # Not directly translatable without a specific input method
    # return command
    pass # Actual implementation depends on how serial input is handled

def serial_motor_target(): # Corresponds to C++ serial_motor_target
    return M0_target_serial # Example, returning M0's target

# --- Cleanup --- 
def deinit_hardware():
    global M0_pwmA, M0_pwmB, M0_pwmC, M1_pwmA, M1_pwmB, M1_pwmC, enable_pin
    print("Deinitializing hardware...")
    DFOC_disable() # Disable motors and set PWMs to 0
    pwm_objs = [M0_pwmA, M0_pwmB, M0_pwmC, M1_pwmA, M1_pwmB, M1_pwmC]
    for pwm in pwm_objs:
        if pwm and hasattr(pwm, 'deinit'): # Check if it's a real PWM object
            pwm.deinit()
    if enable_pin and hasattr(enable_pin, 'deinit'): # Some Pin objects might not have deinit
        pass # enable_pin.deinit() if available/needed
    # I2C objects (S0.wire, S1.wire) could be deinitialized if necessary
    if S0 and S0.wire and hasattr(S0.wire, 'deinit'): S0.wire.deinit()
    if S1 and S1.wire and hasattr(S1.wire, 'deinit'): S1.wire.deinit()
    # ADC objects (CS_M0.adc_A etc) don't typically have deinit in machine.ADC
    print("Hardware deinitialization attempt complete.")

# --- Example Main Execution --- 
if __name__ == '__main__':
    print("DengFOC MicroPython Translation - Example Run")
    try:
        DFOC_Vbus(12.0) # Set bus voltage to 12V

        # Sensor Alignment (important! Use correct Pole Pairs and Direction)
        # Make sure motors can move freely and safely during alignment.
        # DFOC_M0_alignSensor(PP=7, DIR=1)
        # DFOC_M1_alignSensor(PP=7, DIR=1)
        # print("Manual sensor alignment: Set S0_zero_electric_angle and S1_zero_electric_angle if known.")
        # S0_zero_electric_angle = 0.0 # Replace with your calibrated value if skipping alignment
        # S1_zero_electric_angle = 0.0 # Replace with your calibrated value

        # Set default PID parameters (if not already set by DFOC_Vbus or if you want to override)
        # These limits are crucial. Current limit is Uq_max for the current controller.
        # Velocity PID limit is often target current (Iq_max).
        # Angle PID limit is often target velocity (Vel_max).
        DFOC_M0_SET_CURRENT_PID(P=1.0, I=0.5, D=0.0, ramp=10000, limit=voltage_power_supply*0.5) # Limit Uq
        DFOC_M0_SET_VEL_PID(P=0.1, I=0.05, D=0.001, ramp=1000, limit=2.0) # Limit target current (e.g. 2 Amps)
        DFOC_M0_SET_ANGLE_PID(P=5.0, I=0.0, D=0.1, ramp=100, limit=10.0) # Limit target velocity (e.g. 10 rad/s)

        DFOC_enable()
        print("\nRunning FOC loop for M0 (Velocity Control Example)...")
        target_vel_m0 = 1.0 # rad/s
        
        start_run_time = utime.ticks_ms()
        loop_count = 0
        max_loops = 500 # Run for approx 5 seconds if loop is 10ms

        while loop_count < max_loops:
            loop_start_us = utime.ticks_us()
            runFOC() # Update sensors and currents
            
            DFOC_M0_setVelocity(target_vel_m0)
            # DFOC_M0_set_Velocity_Angle(target_angle_m0) # For angle control
            
            # Print status occasionally
            if loop_count % 50 == 0:
                print(f"Loop {loop_count}: M0 Angle={DFOC_M0_Angle():.2f}, Vel={DFOC_M0_Velocity():.2f}, Curr(Iq)={DFOC_M0_Current():.2f}")
            
            loop_end_us = utime.ticks_us()
            loop_duration_us = utime.ticks_diff(loop_end_us, loop_start_us)
            sleep_ms = max(0, 10 - (loop_duration_us // 1000)) # Aim for ~10ms loop time (100Hz)
            utime.sleep_ms(sleep_ms)
            loop_count += 1

        print("\nExample run finished.")

    except KeyboardInterrupt:
        print("\nExecution stopped by user.")
    except Exception as e:
        print(f"An error occurred during main execution: {e}")
        import sys
        sys.print_exception(e)
    finally:
        print("Cleaning up...")
        deinit_hardware()
