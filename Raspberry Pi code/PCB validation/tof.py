# ToF Test Script

# External imports
import pigpio
import adafruit_vl53l0x
import board
import busio
import time

# globals
_pi: pigpio.pi
_vl53: adafruit_vl53l0x.VL53L0X
_i2c: busio.I2C

def init_Pi():
    """Initialize pigpio daemon and return the pi instance.
    """
    global _pi
    print("Initializing pigpio...")
    
    _pi = pigpio.pi()
    if not _pi.connected:
        print("Failed to connect to pigpio daemon")
        while True:
            time.sleep(1)
    
def init_tof(pi: pigpio.pi):
    """Initialize the shared GPIO and sensor hardware used by the system."""
    global _pi, _vl53, _i2c
    
    print("Initializing sensors...")
    _pi = pi

    # board.SCL and board.SDA refer to the Pi's default I2C pins (GPIO 3 and 2).
    # Both sensors share this same I2C bus.
    _i2c = busio.I2C(board.SCL, board.SDA)

    _vl53 = adafruit_vl53l0x.VL53L0X(_i2c)
    
def read_ToF_sensor() -> int:
    """Read the time-of-flight sensor and return the value in millimeters

    Returns:
        int: ToF sensor reading in millimeters
    """
    global _vl53
    return _vl53.range

def main():
    init_Pi()
    init_tof(_pi)
    while True:
        distance = read_ToF_sensor()
        print(f"Distance: {distance} mm")
        time.sleep(1)

if __name__ == "__main__":
    main()