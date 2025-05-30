import machine
import utime

_ADC_VOLTAGE = 3.3  # ADC 电压
_ADC_RESOLUTION = 4095.0  # ADC 分辨率 (对于12位ADC)
_ADC_CONV = _ADC_VOLTAGE / _ADC_RESOLUTION # ADC 计数到电压转换比率

NOT_SET = -12345.0

def _isset(a):
    return a != NOT_SET

class CurrSense:
    def __init__(self, Mot_Num):
        self._Mot_Num = Mot_Num
        self.current_a = 0.0
        self.current_b = 0.0
        self.current_c = 0.0
        
        self.offset_ia = 0.0
        self.offset_ib = 0.0
        self.offset_ic = 0.0
        
        self._shunt_resistor = 0.01  # 分流电阻值 (Ohm)
        self.amp_gain = 50.0         # 电流检测运算放大器增益
        
        # ADC 引脚 (需要根据实际硬件配置)
        # 这些是DengFOC板的默认值，可能需要修改
        if Mot_Num == 0:
            self.pinA_adc_num = 39 # GPIO39 for M0_CS_A
            self.pinB_adc_num = 36 # GPIO36 for M0_CS_B
            self.pinC_adc_num = NOT_SET # 通常只用两相电流检测
        elif Mot_Num == 1:
            self.pinA_adc_num = 35 # GPIO35 for M1_CS_A
            self.pinB_adc_num = 34 # GPIO34 for M1_CS_B
            self.pinC_adc_num = NOT_SET
        else:
            raise ValueError("Mot_Num must be 0 or 1")

        self.adc_A = None
        self.adc_B = None
        self.adc_C = None

        self.volts_to_amps_ratio = 1.0 / self._shunt_resistor / self.amp_gain
        
        # 根据DengFOC V4的配置 (原C++代码注释)
        self.gain_a = self.volts_to_amps_ratio
        self.gain_b = self.volts_to_amps_ratio
        self.gain_c = self.volts_to_amps_ratio
        # 对于DengFOC V3P，增益可能是负的，例如 self.gain_a = -self.volts_to_amps_ratio

    def _read_adc_voltage_inline(self, adc_obj):
        if adc_obj is None:
            return 0.0 # 或者抛出错误，表示ADC未初始化
        raw_adc = adc_obj.read_u16() # read_u16() 通常返回 0-65535 for 16-bit range
        # ESP32 ADC read() or read_uv() might be more direct for voltage or raw value
        # For ESP32, analogRead in Arduino returns 0-4095 for 12-bit ADC.
        # machine.ADC.read_u16() scales to 0-65535. We need to map it back or use read_uv for millivolts.
        # Let's assume read_uv which gives millivolts, then convert to volts.
        # voltage = adc_obj.read_uv() / 1000000.0 # read_uv returns microvolts
        
        # Simpler: use read() which often gives raw ADC value (0-4095 on ESP32 if width is 12-bit)
        # The C++ code uses analogRead() which returns raw ADC value.
        # So, we should aim for that. machine.ADC.read() on ESP32 returns raw value.
        raw_val = adc_obj.read() # Returns raw value (0-4095 for 12-bit ADC on ESP32)
        return raw_val * _ADC_CONV

    def _configure_adc_inline(self):
        # ESP32 ADC pins are typically >= 32 for ADC1 and < 32 for ADC2 (but some overlap)
        # Pin numbers are GPIO numbers.
        self.adc_A = machine.ADC(machine.Pin(self.pinA_adc_num))
        self.adc_B = machine.ADC(machine.Pin(self.pinB_adc_num))
        
        # Standard ESP32 ADC configuration
        self.adc_A.atten(machine.ADC.ATTN_11DB) # For full 0-3.3V range
        self.adc_B.atten(machine.ADC.ATTN_11DB)
        # self.adc_A.width(machine.ADC.WIDTH_12BIT) # Default is 12-bit on most ESP32
        # self.adc_B.width(machine.ADC.WIDTH_12BIT)

        if _isset(self.pinC_adc_num):
            self.adc_C = machine.ADC(machine.Pin(self.pinC_adc_num))
            self.adc_C.atten(machine.ADC.ATTN_11DB)
            # self.adc_C.width(machine.ADC.WIDTH_12BIT)

    def calibrate_offsets(self):
        calibration_rounds = 1000
        offset_sum_a = 0.0
        offset_sum_b = 0.0
        offset_sum_c = 0.0

        print(f"Calibrating current sensor offsets for M{self._Mot_Num}...")
        for _ in range(calibration_rounds):
            offset_sum_a += self._read_adc_voltage_inline(self.adc_A)
            offset_sum_b += self._read_adc_voltage_inline(self.adc_B)
            if _isset(self.pinC_adc_num) and self.adc_C:
                offset_sum_c += self._read_adc_voltage_inline(self.adc_C)
            utime.sleep_ms(1)
        
        self.offset_ia = offset_sum_a / calibration_rounds
        self.offset_ib = offset_sum_b / calibration_rounds
        if _isset(self.pinC_adc_num):
            self.offset_ic = offset_sum_c / calibration_rounds
        print(f"M{self._Mot_Num} Offsets: Ia={self.offset_ia:.3f}V, Ib={self.offset_ib:.3f}V, Ic={self.offset_ic:.3f}V")

    def init(self):
        self._configure_adc_inline()
        self.calibrate_offsets()
        print(f"CurrSense M{self._Mot_Num} initialized.")

    def get_phase_currents(self):
        volts_a = self._read_adc_voltage_inline(self.adc_A)
        volts_b = self._read_adc_voltage_inline(self.adc_B)
        
        self.current_a = (volts_a - self.offset_ia) * self.gain_a
        self.current_b = (volts_b - self.offset_ib) * self.gain_b
        
        if _isset(self.pinC_adc_num) and self.adc_C:
            volts_c = self._read_adc_voltage_inline(self.adc_C)
            self.current_c = (volts_c - self.offset_ic) * self.gain_c
        else:
            # Two-phase sensing: Ic = -(Ia + Ib) - this is often done after Clarke transform
            # For raw currents, if C is not measured, it's often assumed to be 0 or calculated elsewhere.
            # The original C++ sets it to 0 if pinC is not set.
            self.current_c = 0.0

# Example Usage (requires ESP32 or similar with ADC)
if __name__ == '__main__':
    try:
        # Initialize for Motor 0 (M0)
        # Ensure GPIO pins 39 and 36 are available for ADC
        current_sensor_m0 = CurrSense(Mot_Num=0)
        current_sensor_m0.init()

        print("\nReading M0 phase currents for 5 seconds...")
        for i in range(50):
            current_sensor_m0.get_phase_currents()
            print(f"M0 Currents: Ia={current_sensor_m0.current_a:6.3f}A, Ib={current_sensor_m0.current_b:6.3f}A, Ic={current_sensor_m0.current_c:6.3f}A")
            utime.sleep_ms(100)
        
        # Initialize for Motor 1 (M1) if used
        # current_sensor_m1 = CurrSense(Mot_Num=1)
        # current_sensor_m1.init()
        # print("\nReading M1 phase currents for 5 seconds...")
        # for i in range(50):
        #     current_sensor_m1.get_phase_currents()
        #     print(f"M1 Currents: Ia={current_sensor_m1.current_a:6.3f}A, Ib={current_sensor_m1.current_b:6.3f}A, Ic={current_sensor_m1.current_c:6.3f}A")
        #     utime.sleep_ms(100)
            
    except Exception as e:
        print(f"An error occurred: {e}")
