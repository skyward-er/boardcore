#!bin/pyhton3
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import subprocess

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

last = 0
SAMPLES = 1000
# p = subprocess.Popen(["./mav_rx", "18", "115200"], stdout=subprocess.PIPE, bufsize=10)

# def read():
#     while True:
#         line = p.stdout.readline()
#         print(line)



# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    line = sys.stdin.readline()
    line = line[0:-2]
    x, b, y= line.split(',')
    x = float(x)
    y = float(y)
    global last
    if x > last:
        last = x
        xs.append(x)
        ys.append(y)

    print(x, y, b)

    if(len(xs) > SAMPLES):
        xs = xs[-SAMPLES:]
        ys = ys[-SAMPLES:]


    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Time (s)')
    plt.ylabel('Pressure (KPa)')

# read()

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=100)
plt.show()
