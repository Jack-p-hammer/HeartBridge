# buttons.py

# External Imports
import pigpio
import time
import sys

# Button Inputs
NEXT_BTN_PIN = 23
PAUSE_BTN_PIN = 25
# LED Outputs/Button Toggles
NEXT_ENABLE_PIN = 24
PAUSE_ENABLE_PIN = 26

_pi: pigpio.pi
current_time: float
prev_cycle: float

def setup():
    global _pi, current_time, prev_cycle
    
    _pi = pigpio.pi()
    if not _pi.connected:
        print("Failed to connect to pigpio daemon")
        sys.exit()
    # Init  i2c
    print("pi initialized")
    
    # initialize pins
    _pi.set_mode(NEXT_BTN_PIN, pigpio.INPUT)
    _pi.set_mode(PAUSE_BTN_PIN, pigpio.INPUT)
    _pi.set_mode(NEXT_ENABLE_PIN, pigpio.OUTPUT)
    _pi.set_mode(PAUSE_ENABLE_PIN, pigpio.OUTPUT)
    
    # initialize pullups
    _pi.set_pull_up_down(NEXT_BTN_PIN, pigpio.PUD_UP)
    _pi.set_pull_up_down(PAUSE_BTN_PIN, pigpio.PUD_UP)
    
    # initialize cycle
    prev_cycle = time.time()
        
            
def loop():
    global _pi, current_time, prev_cycle
    # Constantly poll button inputs, print if press detected
    if _pi.read(NEXT_BTN_PIN) != 1:
        print("NEXT")
    elif _pi.read(PAUSE_BTN_PIN) != 1:
        print("PAUSE")
    
    # One cycle is 20 sec
    # 5 sec Next ON
    # 5 sec all OFF
    # 5 sec Pause ON
    # 5 sec all OFF
    current_time = time.time()
    cycle_time = current_time - prev_cycle
    if cycle_time >= 15:
        _pi.write(PAUSE_ENABLE_PIN, 0)
        _pi.write(NEXT_ENABLE_PIN, 0)
        prev_cycle = current_time
    elif cycle_time >= 10:
        _pi.write(PAUSE_ENABLE_PIN, 1)
        _pi.write(NEXT_ENABLE_PIN, 0)
    elif cycle_time >= 5:
        _pi.write(PAUSE_ENABLE_PIN, 0)
        _pi.write(NEXT_ENABLE_PIN, 0)
    else:
        _pi.write(PAUSE_ENABLE_PIN, 0)
        _pi.write(NEXT_ENABLE_PIN, 1)
    
    
def main():
    setup()
    while True:
        loop()
        
if __name__ == "__main__":
    main()