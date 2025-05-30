import machine
import utime
import math

_2PI = 6.28318530718

class Sensor_AS5600:
    def __init__(self, Mot_Num, i2c_id=0, scl_pin=22, sda_pin=21, freq=400000):
        self._Mot_Num = Mot_Num
        # AS5600 变量定义
        # self.sensor_direction = 1  # 编码器旋转方向定义 (未使用)
        self.angle_prev = 0.0  # 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
        self.angle_prev_ts = 0  # 上次调用 getAngle 的时间戳
        self.vel_angle_prev = 0.0  # 最后一次调用 getVelocity 时的角度
        self.vel_angle_prev_ts = 0  # 最后速度计算时间戳
        self.full_rotations = 0  # 总圈数计数
        self.vel_full_rotations = 0  # 用于速度计算的先前完整旋转圈数
        
        # 根据 Mot_Num 选择不同的I2C引脚 (这部分在原DengFOC.cpp中初始化)
        # 这里我们允许在构造函数中直接传入，或者使用默认值
        if Mot_Num == 0:
            # S0_I2C.begin(19, 18, 400000UL); # 原C++代码示例
            # 实际引脚应根据硬件配置
            pass # I2C 初始化在 sensor_init 中处理
        elif Mot_Num == 1:
            # S1_I2C.begin(23, 5, 400000UL); # 原C++代码示例
            pass

        self.i2c_id = i2c_id
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.i2c_freq = freq
        self.wire = None # I2C object
        self.AS5600_ADDR = 0x36 # AS5600 I2C address

    def sensor_init(self, wire_obj=None):
        if wire_obj:
            self.wire = wire_obj
        else:
            # 如果没有外部传入 I2C 对象，则尝试内部创建
            # 注意：ESP32上 machine.I2C 的 scl 和 sda 参数是 Pin 对象
            scl = machine.Pin(self.scl_pin)
            sda = machine.Pin(self.sda_pin)
            self.wire = machine.I2C(self.i2c_id, scl=scl, sda=sda, freq=self.i2c_freq)
        
        # 扫描I2C总线，检查设备是否存在
        devices = self.wire.scan()
        if self.AS5600_ADDR not in devices:
            raise OSError(f"AS5600 not found at address {hex(self.AS5600_ADDR)} on I2C bus {self.i2c_id}")

        utime.sleep_ms(500)
        self.get_sensor_angle() 
        utime.sleep_us(1)
        self.vel_angle_prev = self.get_sensor_angle() 
        self.vel_angle_prev_ts = utime.ticks_us()
        utime.sleep_ms(1) # 原C++是delay(1)
        self.get_sensor_angle() 
        utime.sleep_us(1)
        self.angle_prev = self.get_sensor_angle() 
        self.angle_prev_ts = utime.ticks_us()

    def get_sensor_angle(self):
        angle_reg_msb = 0x0C # Raw angle register
        
        try:
            self.wire.writeto(self.AS5600_ADDR, bytes([angle_reg_msb]), False)
            read_array = self.wire.readfrom(self.AS5600_ADDR, 2)
        except OSError as e:
            print(f"I2C communication error: {e}")
            return self.angle_prev # Return last known angle on error
            
        # 原C++代码中的位处理逻辑，直接翻译
        # int _bit_resolution=12;
        # int _bits_used_msb=11-7; // This seems to be (12-1)-7 = 4 for MSB part of 12-bit raw angle
        # float cpr = pow(2, _bit_resolution);
        # int lsb_used = _bit_resolution - _bits_used_msb;

        # uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
        # uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
  
        # readValue = ( readArray[1] &  lsb_mask );
        # readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
        # return (readValue/ (float)cpr) * _2PI;

        # AS5600 datasheet: ANGLE (0x0E, 0x0F) or RAW ANGLE (0x0C, 0x0D)
        # The raw angle is in 0x0C (MSB) and 0x0D (LSB)
        # The data is 12-bit, so (read_array[0] << 8 | read_array[1]) & 0x0FFF
        # However, the C++ code uses 0x0C as the register to read from, which is RAW ANGLE MSB.
        # It then reads 2 bytes. So it's reading RAW_ANGLE_MSB (0x0C) and RAW_ANGLE_LSB (0x0D).
        
        # Let's re-evaluate the C++ bit manipulation based on typical AS5600 register reads.
        # If angle_reg_msb = 0x0C, it reads from 0x0C and 0x0D.
        # readArray[0] = content of 0x0C (MSB of raw angle)
        # readArray[1] = content of 0x0D (LSB of raw angle)
        # Raw angle is 12 bits (0-4095). MSB register (0x0C) contains bits 11..8. LSB register (0x0D) contains bits 7..0.
        # So, raw_value = (readArray[0] << 8 | readArray[1]) & 0x0FFF (if reading from 0x0C and 0x0D directly)
        # The C++ code's bit manipulation seems specific or perhaps adapted for a particular I2C read sequence or interpretation.
        # For simplicity and standard AS5600 reading, let's use the direct 12-bit interpretation from registers 0x0E and 0x0F (scaled angle)
        # or 0x0C and 0x0D (raw angle).
        # The original code reads from 0x0C (RAW ANGLE MSB) and then reads 2 bytes.
        # This implies it's reading 0x0C and 0x0D.
        # readArray[0] = value from 0x0C (bits 11:8 of raw angle)
        # readArray[1] = value from 0x0D (bits 7:0 of raw angle)
        # So, raw_value = (readArray[0] << 8 | readArray[1]) & 0x0FFF
        
        # The C++ code's bit manipulation: 
        # _bit_resolution = 12
        # _bits_used_msb = 4 (11-7)
        # cpr = 4096
        # lsb_used = 8 (12-4)
        # lsb_mask = (2 << 8) - 1 = 511 (0x1FF) -> this should be (1 << lsb_used) -1 = 255 (0xFF)
        # msb_mask = (2 << 4) - 1 = 31 (0x1F) -> this should be (1 << _bits_used_msb) -1 = 15 (0x0F)
        # readValue = (readArray[1] & lsb_mask)  -> (readArray[1] & 0xFF) (Correct for LSB)
        # readValue += ((readArray[0] & msb_mask) << lsb_used) -> ((readArray[0] & 0x0F) << 8) (Correct for MSB)
        # This is the standard way to combine MSB (upper 4 bits of 12) and LSB (lower 8 bits of 12)
        
        # Corrected interpretation of C++ logic for 12-bit value from two 8-bit registers:
        # Assuming readArray[0] is MSB and readArray[1] is LSB of the 12-bit angle data
        # The AS5600 outputs angle in registers 0x0E (high byte) and 0x0F (low byte).
        # These are bits 11:8 and 7:0 of the 12-bit result.
        # So, raw_value = (read_array[0] << 8 | read_array[1]) & 0x0FFF
        # The original C++ code uses register 0x0C (RAW_ANGLE_H) and reads two bytes.
        # So it reads RAW_ANGLE_H (0x0C) and RAW_ANGLE_L (0x0D).
        # RAW_ANGLE_H contains bits 11..8. RAW_ANGLE_L contains bits 7..0.
        raw_value = (read_array[0] << 8 | read_array[1]) & 0x0FFF
        
        cpr = 4096.0 # 2^12
        return (raw_value / cpr) * _2PI

    def sensor_update(self):
        val = self.get_sensor_angle()
        current_ts = utime.ticks_us()
        
        # Handle potential rollover of ticks_us for d_angle calculation if angle_prev_ts is much older
        # However, d_angle is based on sensor values, not timestamps directly here.
        self.angle_prev_ts = current_ts # Update timestamp regardless
        
        d_angle = val - self.angle_prev
        
        # Wrap d_angle to the range [-PI, PI) for correct full_rotations tracking
        if d_angle > math.pi:
            d_angle -= _2PI
        elif d_angle < -math.pi:
            d_angle += _2PI
            
        # Original C++ condition: if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
        # This logic seems to detect a wrap-around if the change is very large.
        # A more standard way for sensor wrap-around (e.g. 0 to 2PI or 2PI to 0):
        # If d_angle is large and positive (e.g. val near 0, angle_prev near 2PI), it means angle decreased across wrap.
        # If d_angle is large and negative (e.g. val near 2PI, angle_prev near 0), it means angle increased across wrap.
        
        # Let's stick to the original logic's intent for now, which seems to be:
        # if change is more than 80% of a full circle, assume it's a wrap-around in the opposite direction of the small change.
        # Example: val = 0.1, angle_prev = 6.2. d_angle_raw = 0.1 - 6.2 = -6.1. abs(d_angle_raw) > 0.8 * 2PI.
        # d_angle_raw > 0 is false. So full_rotations += 1. (Correct, moved from near 2PI to near 0, one rotation forward)
        # Example: val = 6.2, angle_prev = 0.1. d_angle_raw = 6.2 - 0.1 = 6.1. abs(d_angle_raw) > 0.8 * 2PI.
        # d_angle_raw > 0 is true. So full_rotations += -1. (Correct, moved from near 0 to near 2PI, one rotation backward)
        # The original C++ logic for full_rotations seems correct for its specific wrap detection.
        
        # Re-evaluating the original C++ logic for full_rotations:
        # float d_angle = val - angle_prev; // This d_angle is not wrapped yet.
        # if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
        # Example 1: angle_prev = 0.1, val = 6.2 (moved backward across zero)
        # d_angle = 6.2 - 0.1 = 6.1. abs(6.1) > 0.8*6.28 (True). d_angle > 0 (True). full_rotations += -1. Correct.
        # Example 2: angle_prev = 6.2, val = 0.1 (moved forward across zero)
        # d_angle = 0.1 - 6.2 = -6.1. abs(-6.1) > 0.8*6.28 (True). d_angle > 0 (False). full_rotations += 1. Correct.
        # So the original C++ logic for full_rotations is fine.
        raw_d_angle = val - self.angle_prev # Use the non-wrapped d_angle for this check
        if abs(raw_d_angle) > (0.8 * _2PI):
            if raw_d_angle > 0:
                self.full_rotations -= 1
            else:
                self.full_rotations += 1
        
        self.angle_prev = val
        # self.angle_prev_ts was already updated at the start of the method.

    def get_mechanical_angle(self):
        return self.angle_prev

    def get_angle(self):
        # Ensure sensor_update() has been called recently enough for angle_prev to be current
        return float(self.full_rotations) * _2PI + self.angle_prev

    def get_velocity(self):
        current_ts = self.angle_prev_ts # Use the timestamp from the last sensor_update
        
        # Calculate time difference, handling potential ticks_us rollover
        Ts = utime.ticks_diff(current_ts, self.vel_angle_prev_ts) * 1e-6
        
        if Ts <= 0: # Can happen if called too quickly or due to timer precision
            Ts = 1e-3 # Default to 1ms if delta_t is invalid
        
        # Angle difference, accounting for full rotations
        angle_diff = (float(self.full_rotations - self.vel_full_rotations) * _2PI) + (self.angle_prev - self.vel_angle_prev)
        
        vel = angle_diff / Ts
        
        # Save variables for next velocity calculation
        self.vel_angle_prev = self.angle_prev
        self.vel_full_rotations = self.full_rotations
        self.vel_angle_prev_ts = current_ts # Use the timestamp from the last sensor_update
        
        return vel

# Example Usage (requires an AS5600 sensor connected to I2C0, SCL=Pin(19), SDA=Pin(18) for Mot_Num=0 on ESP32)
if __name__ == '__main__':
    # ESP32 specific pins for I2C bus 0
    # scl_pin_s0 = 19 
    # sda_pin_s0 = 18
    # sensor0 = Sensor_AS5600(Mot_Num=0, i2c_id=0, scl_pin=scl_pin_s0, sda_pin=sda_pin_s0)
    
    # Generic pins, ensure these are correct for your board
    # scl_pin_generic = machine.Pin.board.SCL # Or specific GPIO number
    # sda_pin_generic = machine.Pin.board.SDA # Or specific GPIO number
    # sensor0 = Sensor_AS5600(Mot_Num=0, i2c_id=0, scl_pin=scl_pin_generic, sda_pin=sda_pin_generic)

    # Placeholder pins if you don't have a specific board context here
    # You'll need to replace these with actual pin numbers for your ESP32 or other MicroPython board
    # For example, on a generic ESP32 board: SCL=GPIO22, SDA=GPIO21 for I2C bus 0 or 1
    # Or SCL=GPIO19, SDA=GPIO18 for I2C bus 0 (as in original C++ for S0)
    
    print("Attempting to initialize AS5600 sensor...")
    print("Please ensure your AS5600 is connected to the correct I2C pins.")

    try:
        # Example for S0 on ESP32 as per original C++ comments (I2C0: SCL=19, SDA=18)
        # Note: machine.Pin numbers are GPIO numbers.
        sensor0 = Sensor_AS5600(Mot_Num=0, i2c_id=0, scl_pin=19, sda_pin=18)
        sensor0.sensor_init()
        print("Sensor initialized.")

        for _ in range(20):
            sensor0.sensor_update()
            angle_rad = sensor0.get_angle()
            angle_deg = math.degrees(angle_rad)
            velocity_rps = sensor0.get_velocity()
            velocity_dps = math.degrees(velocity_rps)
            
            print(f"Angle: {angle_rad:.2f} rad ({angle_deg:.2f} deg), Velocity: {velocity_rps:.2f} rad/s ({velocity_dps:.2f} deg/s)")
            utime.sleep_ms(100)
            
    except OSError as e:
        print(f"Error during AS5600 example: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
