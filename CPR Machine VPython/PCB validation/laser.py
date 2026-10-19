# laser.py

# External Imports
import pigpio
import time
import sys

# Laser PWM Output
LASER_PIN = 12
# Duty Cycle Scale: 0-1M
LASER_PWM_SCALE: int = 1000000
# For now, 25 kHz PWM
LASER_PWM_FREQUENCY:int = 25000

_pi: pigpio.pi

def setup():
    global _pi
    
    _pi = pigpio.pi()
    if not _pi.connected:
        print("Failed to connect to pigpio daemon")
        sys.exit()
    # Init  i2c
    print("pi initialized")
    
    # Start with solid laser
    # Duty cycle ranges from 0-1M
    # Initialize to off
    _pi.hardware_PWM(LASER_PIN, 0, 0*LASER_PWM_SCALE)
    
    # First two tests: 100% Power, 50% Power
    _pi.hardware_PWM(LASER_PIN, LASER_PWM_FREQUENCY, 1*LASER_PWM_FREQUENCY)
    time.sleep(5)
    _pi.hardware_PWM(LASER_PIN, 0, 0*LASER_PWM_SCALE)
    time.sleep(5)
    
    _pi.hardware_PWM(LASER_PIN, LASER_PWM_FREQUENCY, int(0.5*LASER_PWM_FREQUENCY))
    time.sleep(5)
    _pi.hardware_PWM(LASER_PIN, 0, 0*LASER_PWM_SCALE)
    time.sleep(5)
        
            
def loop():
    global _pi
    # Sweep PWM duty cycles
    # Rate: 0.5%/step, 200 step/sec
    # Sweep between 25% and 75%
    for duty_cycle in range(int(0.1*LASER_PWM_SCALE), int(0.8*LASER_PWM_SCALE), int(0.001*LASER_PWM_SCALE)):
        _pi.hardware_PWM(LASER_PIN, LASER_PWM_FREQUENCY, duty_cycle)
        time.sleep(0.005)
        
    
    
def main():
    setup()
    
    while True:
        loop()
        
if __name__ == "__main__":
    main()