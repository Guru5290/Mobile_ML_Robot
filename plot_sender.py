# pi_sender.py
import socket
import time
from serial import Serial
from serial.tools.list_ports import comports
import serial.tools.list_ports

# HOST = "192.168.43.78"  # Replace with your PC's IP address
# HOST = "Big-Konstantin.local"  # Replace with your PC's IP address
HOST = "g.local"  # Replace with your PC's IP address
PORT = 12345


def init_serial():
    TARGET_HWID = "PID=2341:0042"  # for an arduino mega
    TARGET_LOCATION = "1.3"  # for usb to ttl converter
    ports = comports()
    port_name = None
    for port in ports:
        # if TARGET_HWID in port.hwid:
        if TARGET_LOCATION in port.hwid:
            port_name = port.device

    print("Available devices are: ")
    ports = comports()
    for port in ports:
        print(
            f"Device: {port.device}, Description: {port.description}, HWID: {port.hwid}"
        )

    if not port_name:
        print("Arduino Mega not found!")
        quit()

    global ser
    ser = Serial(port_name, baudrate=57600, timeout=1)
    ser.write(b"i\n"*10) # send a couple to make sure the message gets there


def main():
    init_serial()
    global s
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting to PC...")
        s.connect((HOST, PORT))
        print("Connected!")

        while True:
            try:
                read = ser.read_until().decode().strip() + "\n"
                # read_conv = parse(read)
            except UnicodeDecodeError:
                pass

            s.sendall(read.encode())
            # time.sleep(0.01)  # Send every 10ms


if __name__ == "__main__":
    try:
        main()
    except:
    # except KeyboardInterrupt:
        ser.write(b"z\n\r" * 10)  # send a couple to ensure the message gets there
        ser.close()

        # s.shutdown(socket.SHUT_RDWR)
        s.close()

        exit()
