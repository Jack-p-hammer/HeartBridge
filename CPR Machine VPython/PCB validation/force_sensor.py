# force_sensor.py

# External imports
import pigpio
import busio
import board
import time
import sys

# I2C address for the analog-to-digital converter. TODO: Use the I2C_scanner.py script to verify this!
ADC_ADDR = 0x48   # A0 variant: device code 1001 + address bits 000
ADC_VDD: float = 4.94  # 5V reference voltage for the ADC, used to convert the raw ADC reading to a voltage value
ADC_BITS: int = 10 

_pi: pigpio.pi
_i2c: busio.I2C

def setup():
    global _i2c, _pi
    
    _pi = pigpio.pi()
    if not _pi.connected:
        print("Failed to connect to pigpio daemon")
        sys.exit()
    # Init  i2c
    try:
        _i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"Failed to initialize I2C bus: {e}")
        sys.exit()
        
    print("i2c initialized")
        

def read_ADC() -> int:
    """Read the analog-to-digital converter (ADC) and return the raw value

    The chip sends 2 bytes:
      Byte 1: 0 0 0 0 0 0 D9 D8   (upper 6 bits are don't-care/zero)
      Byte 2: D7 D6 D5 D4 D3 D2 D1 D0

    So the 10-bit value is ((byte1 & 0x03) << 8) | byte2

    Returns:
        int: ADC reading in raw counts (0-1023)
    """
    global _i2c

    result: bytearray = bytearray(2)

    # Read ADC
    _i2c.readfrom_into(ADC_ADDR, result)
    raw: int = ((result[0] & 0x03) << 8) | result[1]
    
    return raw
   
    
def read_force_sensor() -> float:
    """Read the force sensor and return the value in Newtons

    Returns:
        float: Force sensor reading in Newtons
    """
    raw_reading: int = read_ADC()
    raw_voltage: float = (raw_reading / (2^ADC_BITS)) * ADC_VDD

    # TODO: Calibrate force sensor
    # Placeholder conversion: 1V = 100N
    force_newtons: float = raw_voltage * 100.0

    return force_newtons

    
def loop():
    value: float = read_force_sensor()
    print(value)
    time.sleep(0.125)
    
    
    
def main():
    setup()
    
    while True:
        loop()
        
if __name__ == "__main__":
    main()