from math import acos, sin, cos, sqrt

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, TextBox

# Global variable for the animation
ani = None

def get_path_points(start_point, end_point, discretization):
    path_points = []
    x_start, y_start = start_point
    x_end, y_end = end_point

def get_path_points(start, end, discretization):
    """Compute linear path points between start and end."""
    x_start, y_start = start
    x_end, y_end = end
    x_values = np.linspace(x_start, x_end, discretization)
    y_values = np.linspace(y_start, y_end, discretization)
    return list(zip(x_values, y_values))


def get_angles(point, l_1, l_2):
    """Calculate joint angles for the manipulator to reach a point."""
    x, y = point
    cos_theta2 = (x ** 2 + y ** 2 - l_1 ** 2 - l_2 ** 2) / (2 * l_1 * l_2)
    sin_theta2 = np.sqrt(1 - cos_theta2 ** 2)

    theta2_1 = np.arctan2(sin_theta2, cos_theta2)
    theta2_2 = np.arctan2(-sin_theta2, cos_theta2)

    theta1_1 = np.arctan2(y, x) - np.arctan2(l_2 * np.sin(theta2_1), l_1 + l_2 * np.cos(theta2_1))
    theta1_2 = np.arctan2(y, x) - np.arctan2(l_2 * np.sin(theta2_2), l_1 + l_2 * np.cos(theta2_2))

    return [(theta1_1, theta2_1), (theta1_2, theta2_2)]


def update_plot(frame, path_points, l_1, l_2, lines):
    """Update the plot for each frame."""
    angles = get_angles(path_points[frame], l_1, l_2)
    for i, (theta1, theta2) in enumerate(angles):
        x1, y1 = l_1 * np.cos(theta1), l_1 * np.sin(theta1)
        x2, y2 = x1 + l_2 * np.cos(theta1 + theta2), y1 + l_2 * np.sin(theta1 + theta2)
        lines[i].set_data([0, x1, x2], [0, y1, y2])
    return lines


def get_middle_points(path_1, path_2, l_1):
    middle_points_1 = calculate_points(path_1, l_1)
    middle_points_2 = calculate_points(path_2, l_1)

    return middle_points_1, middle_points_2

def main():
    # Input parameters
    l_1, l_2 = map(float, input('L1 L2: ').split())
    start_point = tuple(map(float, input('X1 Y1: ').split()))
    end_point = tuple(map(float, input('X2 Y2: ').split()))

    # Create plot
    fig, ax = plt.subplots()
    ax.set_xlim([-l_1 - l_2, l_1 + l_2])
    ax.set_ylim([-l_1 - l_2, l_1 + l_2])
    ax.set_title("2D Planar Manipulator")
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.grid(True)

    # Create lines for the manipulator's links
    lines = [ax.plot([], [], 'o-')[0] for _ in range(2)]

    # Create path points
    path_points = get_path_points(start_point, end_point, 1000)

    discretization_slider.on_changed(update_discretization)

    # Animation
    global ani
    ani = FuncAnimation(fig, update_plot,
                        frames=len(path_points),
                        interval=1000 / slider.val,
                        fargs=(path_points, l_1, l_2, lines),
                        blit=True)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
