from machine import Pin, ADC
import time

# 定义常量
_ADC_VOLTAGE = 3.3            # ADC 电压
_ADC_RESOLUTION = 4095.0      # ADC 分辨率
_ADC_CONV = _ADC_VOLTAGE / _ADC_RESOLUTION  # ADC 计数到电压转换比率
NOT_SET = -12345.0

def _isset(a):
    return a != NOT_SET

class CurrSense:
    def __init__(self, Mot_Num):
        self._Mot_Num = Mot_Num
        if Mot_Num == 0:
            self.pinA = 39
            self.pinB = 36
            # pinC 未定义，留空
            self._shunt_resistor = 0.01
            self.amp_gain = 50
            
            self.volts_to_amps_ratio = 1.0 / self._shunt_resistor / self.amp_gain  # volts to amps
            
            # DengFOC V4
            self.gain_a = self.volts_to_amps_ratio
            self.gain_b = self.volts_to_amps_ratio
            self.gain_c = self.volts_to_amps_ratio
            
        if Mot_Num == 1:
            self.pinA = 35
            self.pinB = 34
            # pinC 未定义，留空
            self._shunt_resistor = 0.01
            self.amp_gain = 50
            
            self.volts_to_amps_ratio = 1.0 / self._shunt_resistor / self.amp_gain  # volts to amps
            
            # DengFOC V4
            self.gain_a = self.volts_to_amps_ratio
            self.gain_b = self.volts_to_amps_ratio
            self.gain_c = self.volts_to_amps_ratio

        # 初始化成员变量
        self.current_a = 0.0
        self.current_b = 0.0
        self.current_c = 0.0
        self.offset_ia = 0.0
        self.offset_ib = 0.0
        self.offset_ic = 0.0
        self.pinC = NOT_SET  # 默认未设置

    def readADCVoltageInline(self, pinA):
        adc = ADC(Pin(pinA))
        adc.atten(ADC.ATTN_11DB)  # 设置衰减以支持 0-3.3V 范围
        raw_adc = adc.read()      # 读取 ADC 值（0-4095）
        return raw_adc * _ADC_CONV

    def configureADCInline(self, pinA, pinB, pinC):
        Pin(pinA, Pin.IN)
        Pin(pinB, Pin.IN)
        if _isset(pinC):
            Pin(pinC, Pin.IN)
        self.pinA = pinA
        self.pinB = pinB
        self.pinC = pinC

    def calibrateOffsets(self):
        calibration_rounds = 1000

        # 查找零电流时的电压
        self.offset_ia = 0
        self.offset_ib = 0
        self.offset_ic = 0

        # 读数1000次
        for i in range(calibration_rounds):
            self.offset_ia += self.readADCVoltageInline(self.pinA)
            self.offset_ib += self.readADCVoltageInline(self.pinB)
            if _isset(self.pinC):
                self.offset_ic += self.readADCVoltageInline(self.pinC)
            time.sleep_ms(1)  # 延时 1ms

        # 求平均值，得到误差
        self.offset_ia = self.offset_ia / calibration_rounds
        self.offset_ib = self.offset_ib / calibration_rounds
        if _isset(self.pinC):
            self.offset_ic = self.offset_ic / calibration_rounds

    def init(self):
        # 配置引脚
        self.configureADCInline(self.pinA, self.pinB, self.pinC)
        # 校准
        self.calibrateOffsets()

    def getPhaseCurrents(self):
        self.current_a = (self.readADCVoltageInline(self.pinA) - self.offset_ia) * self.gain_a  # amps
        self.current_b = (self.readADCVoltageInline(self.pinB) - self.offset_ib) * self.gain_b  # amps
        self.current_c = 0 if not _isset(self.pinC) else (self.readADCVoltageInline(self.pinC) - self.offset_ic) * self.gain_c  # amps
'''        
# 示例用法
curr_sense = CurrSense(0)  # 初始化为 Mot_Num = 0
curr_sense.init()          # 配置并校准
curr_sense.getPhaseCurrents()  # 获取相电流
print(curr_sense.current_a, curr_sense.current_b, curr_sense.current_c)
'''
