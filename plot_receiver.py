# pc_receiver
import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 12345

values = [0, 0, 0, 0]
x_current = 0
xs = []
ys1 = []
ys2 = []
ys3 = []
ys4 = []

max_datapoints = 150


def parse(s: str, sep=" "):
    """Converts delimitted string to list of floats"""
    items = s.split(sep=" ")

    items_conv = [0 for _ in range(len(items))]

    for x in range(len(items)):
        try:
            items_conv[x] = float(items[x])
        except ValueError:
            items_conv[x] = 0

    return items_conv


def sync_lengths(*lists):
    min_len = min(len(lst) for lst in lists)
    return [lst[-min_len:] for lst in lists]


def animate(i):
    global ax1, ax2, xs, ys1, ys2, ys3, ys4, x_current, values

    if len(values) == 0:
        return

    x_current += 1

    xs.append(x_current)

    try:
        ys1.append(values[0])
        ys2.append(values[1])
        ys3.append(values[2])
        ys4.append(values[3])

    except IndexError as e:
        try:
            ys1.append(ys1[-1])
            ys2.append(ys2[-1])
            ys3.append(ys3[-1])
            ys4.append(ys4[-1])
        except IndexError:
            ys1.append(0)
            ys2.append(0)
            ys3.append(0)
            ys4.append(0)

    while (
        len(xs)
    ) > max_datapoints:  # can just pop one as we'll synchronize the lengths in the next line
        xs.pop(0)

    xs, ys1, ys2, ys3, ys4 = sync_lengths(xs, ys1, ys2, ys3, ys4)

    ax1.clear()
    ax2.clear()

    ax2.plot(xs, ys1, label="Dist. x", color='g')
    ax1.plot(xs, ys2, label="Acc. x")
    ax1.plot(xs, ys3, label="Acc. y")
    ax1.plot(xs, ys4, label="Acc. z")

    ax1.legend(loc='center left')
    ax2.legend(loc='center left')


def main():
    global ax1, ax2, fig, values

    style.use("fivethirtyeight")

    # fig = plt.figure()
    # ax1 = fig.add_subplot(1, 1, 1)
    # ax2 = ax1.twinx()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))  # Wide and tall
    global s
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print("Waiting for connection...")
        conn, addr = s.accept()
        print(f"Connected by {addr}")

        def wireless_reader():
            global values
            with conn:
                buffer = ""
                while True:
                    data = conn.recv(1024).decode()
                    if not data:
                        break

                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        values = parse(line)
                        # print("Parsed:", values)

        import threading

        threading.Thread(target=wireless_reader, daemon=True).start()

        ani = animation.FuncAnimation(fig, animate, interval=10, cache_frame_data=False)
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:

        # s.shutdown(socket.SHUT_RDWR)
        s.close()

        exit()
