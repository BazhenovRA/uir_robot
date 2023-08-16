import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

# Create main figure and axis
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)  # Make space for the slider

# Initial plot
x = np.arange(0, 2 * np.pi, 0.01)
line, = ax.plot(x, np.sin(x))

# Slider
ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
speed_slider = Slider(ax_slider, 'Speed', 0.1, 2.0, valinit=1.0)


# Animation update function
def animate(i):
    line.set_ydata(np.sin(x + speed_slider.val * i / 50))
    return line,


ani = animation.FuncAnimation(fig, animate, interval=20, blit=True)

plt.show()
