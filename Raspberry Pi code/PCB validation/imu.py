# imu.py

# External imports
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import pigpio
import busio
import board
import time
import sys

# I2C address for the BNO085 IMU. TODO: Use the I2C_scanner.py script to verify this!
IMU_ADDR = 0x4A   # Default address (SA0 low), 0x4B if SA0 pulled high

_pi: pigpio.pi
_i2c: busio.I2C
_bno: BNO08X_I2C

def setup():
    global _i2c, _pi, _bno

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

    # Init IMU
    try:
        _bno = BNO08X_I2C(_i2c, address=IMU_ADDR)
        # Accelerometer is the report used for kneel detection in src/sensing.py
        _bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        _bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        _bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
    except Exception as e:
        print(f"Failed to initialize BNO085 IMU: {e}")
        sys.exit()

    print("IMU initialized")


def loop():
    global _bno

    try:
        accel_x, accel_y, accel_z = _bno.acceleration  # m/s^2
        gyro_x, gyro_y, gyro_z = _bno.gyro  # rad/s
        quat_i, quat_j, quat_k, quat_real = _bno.quaternion
    except Exception as e:
        print(f"IMU read failed: {e}")
        time.sleep(0.125)
        return

    print(f"Accel (m/s^2): X={accel_x:7.3f} Y={accel_y:7.3f} Z={accel_z:7.3f} | "
          f"Gyro (rad/s): X={gyro_x:7.3f} Y={gyro_y:7.3f} Z={gyro_z:7.3f} | "
          f"Quat: I={quat_i:6.3f} J={quat_j:6.3f} K={quat_k:6.3f} Real={quat_real:6.3f}")
    time.sleep(0.125)


def main():
    setup()

    while True:
        loop()

if __name__ == "__main__":
    main()
