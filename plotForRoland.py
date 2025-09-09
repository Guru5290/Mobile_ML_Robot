import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from serial import Serial
from serial.tools.list_ports import comports


read_conv = [0, 0, 0 ,0]
x_current = 0
xs = []
ys1 = []
ys2 = []
ys3 = []
ys4 = []

max_datapoints = 150


def init_serial():
    TARGET_HWID = "PID=2341:0042"  # for an arduino mega
    ports = comports()
    port_name = None
    for port in ports:
        if TARGET_HWID in port.hwid:
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


def parse(s: str, sep=" "):
    """Converts delimitted string to list of floats"""
    items = s.split(sep=" ")
    items_conv = [0 for _ in range(len(items))]

    for x in range(len(items)):
        if items[x].isdecimal() or '-' in items[x]: # the second condition is to accomodate negative numbers
            try:
                items_conv[x] = float(items[x])
            except ValueError:
                items_conv[x] = 0

    return items_conv



def sync_lengths(*lists):
    min_len = min(len(lst) for lst in lists)
    return [lst[:min_len] for lst in lists]


def animate(i):
    global xs, ys1,ys2,ys3,ys4, x_current, read_conv

    if len(read_conv) == 0:
        return

    x_current += 1

    xs.append(x_current)
    try:
        ys1.append(read_conv[0])
        ys2.append(read_conv[1])
        # ys3.append(read_conv[2])
        # ys4.append(read_conv[3])
    except IndexError as e:
        print(e)
        ys1.append(0)
        ys2.append(0)
        # ys3.append(0)
        # ys4.append(0)

    while (len(xs)) > max_datapoints:
        xs.pop(0)
        ys1.pop(0)
        ys2.pop(0)
        # ys3.pop(0)
        # ys4.pop(0)

    
    ys1, ys2 = sync_lengths(ys1, ys2)
    # ys1, ys2, ys3, ys4 = sync_lengths(ys1, ys2, ys3, ys4)
    

    ax1.clear()
    ax1.plot(xs, ys1, label="Acc. x")
    ax1.plot(xs, ys2, label="Acc. y")
    # ax1.plot(xs, ys1, label="Acc. z")
    # ax1.plot(xs, ys1, label="Dist. x")


def main():
    global ax1, fig, read_conv

    style.use("fivethirtyeight")
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    init_serial()

    def serial_reader():
        global read_conv
        while True:
            try:
                read = ser.read_until().decode().strip()
                read_conv = parse(read)
            except UnicodeDecodeError:
                pass
            # print(f"Received: {read_conv}")

    import threading
    threading.Thread(target=serial_reader, daemon=True).start()

    ani = animation.FuncAnimation(fig, animate, interval=10, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        ser.close()
