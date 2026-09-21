"""
I2C bus scanner using busio (Adafruit Blinka / CircuitPython).

busio.I2C actually has a built-in scan() method that does the address
sweep for us, so this is mostly about locking/unlocking the bus and
formatting the output nicely.

Useful for confirming the address of an MCP3021 ADC before talking to it.
"""

import board
import busio


def scan_bus():
    # Initialize I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)

    # busio requires you to explicitly grab the bus lock before doing
    # any transactions, including scanning. Other code/threads can't
    # use the bus while you hold the lock.
    while not i2c.try_lock():
        pass

    try:
        print("Scanning I2C bus...\n")

        # i2c.scan() returns a list of 7-bit addresses (as ints) that ACKed.
        found = i2c.scan()

        # Print as a standard 16x8 address grid
        print("     " + " ".join(f"{x:02x}" for x in range(16)))
        for row in range(0, 0x80, 16):
            line = f"{row:02x}: "
            for col in range(16):
                addr = row + col
                if addr < 0x03 or addr > 0x77:
                    line += "   "  # reserved address range, skip
                elif addr in found:
                    line += f"{addr:02x} "
                else:
                    line += "-- "
            print(line)

        print("\nDevices found:", [hex(a) for a in found] if found else "none")

    finally:
        # Always release the lock so other code can use the bus again.
        i2c.unlock()

    return found


if __name__ == "__main__":
    scan_bus()
