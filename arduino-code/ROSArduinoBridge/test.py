from serial import Serial
import time
import random


PORT = "COM35"
BAUDRATE = 115200

# RANGE = (-1023, 1023)
RANGE = (-60, 60)

INTERVAL = 1

def main():
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        while True:
            w = random.randint(*RANGE)
            y = random.randint(*RANGE)
            x = random.randint(*RANGE)
            z = random.randint(*RANGE)
            # message = f"o {w} {x} {y} {z}\r"
            message = f"m {w} {x} {y} {z}\r"
            ser.write(message.encode("utf-8"))
            print(message.strip())
            time.sleep(INTERVAL)


if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt:
        print("Ctrl+C terminating...")
