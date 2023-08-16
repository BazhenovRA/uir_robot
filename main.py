from math import acos, sin, cos, sqrt

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, TextBox


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


def make_lines(path_points, middle_points_1, middle_points_2):
    line_1 = [((0, 0), middle_points_1[i], path_points[i]) for i in range(len(path_points))]
    line_2 = [((0, 0), middle_points_2[i], path_points[i]) for i in range(len(path_points))]

    return line_1, line_2


def show_plot(frames: int, ax, line_1, line_2):
    x_points_1, y_points_1 = zip(*line_1[frames - 1])
    x_points_2, y_points_2 = zip(*line_2[frames - 1])

    a, = ax.plot(x_points_1, y_points_1, color='green', marker='o', markersize=7)
    b, = ax.plot(x_points_2, y_points_2, color='red', marker='o', markersize=7)

    return [a, b]



def update_speed(speed_slider):
    speed = speed_slider.val
    ani.event_source.interval = speed


def update_discretization(l_1, l_2, start_point, end_point, discretization) -> None:


    global path_points, all_angles, filtered_angles_1, filtered_angles_2, middle_points_1, middle_points_2, line_1, line_2, path_1, path_2
    path_points = get_path_points(start_point, end_point, discretization)
    all_angles = get_angles(path_points, l_1, l_2)
    path_1, path_2 = filter_angles(all_angles)
    filtered_angles_1, filtered_angles_2 = filter_angles(all_angles)
    middle_points_1, middle_points_2 = get_middle_points(filtered_angles_1, filtered_angles_2, l_1)
    line_1, line_2 = make_lines(path_points, middle_points_1, middle_points_2)

    global ani
    fig, ax = plt.subplots()
    ax.set_xlim([-1, 15])
    ax.set_ylim([-1, 15])

    plt.title("Манипулятор на плоскости")
    plt.xlabel('Ось Х')
    plt.ylabel('Ось У')
    plt.legend(['line_1', ' line_2'])
    plt.grid(which='major')


    ani = FuncAnimation(fig, show_plot, frames=len(path_points), fargs=(ax, line_1, line_2),
                        interval=50, blit=True, repeat=True, repeat_delay=300)

    plt.subplots_adjust(bottom=0.35)




    speed_slider_ax = plt.axes([0.2, 0.20, 0.65, 0.03])
    speed_slider = Slider(speed_slider_ax, 'Speed', 1, 100, valinit=50)
    speed_slider.on_changed(update_speed)

    discretization_slider_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
    discretization_slider = Slider(discretization_slider_ax, 'Discretization', 1, 100, valinit=100)

    discretization_slider.on_changed(update_discretization)

    start_x_textbox_ax = plt.axes([0.35, 0.08, 0.1, 0.04])
    start_x_textbox = TextBox(start_x_textbox_ax, 'start_x ', initial='0')
    start_y_textbox_ax = plt.axes([0.35, 0.02, 0.1, 0.04])
    start_y_textbox = TextBox(start_y_textbox_ax, 'start_y ', initial='0')
    end_x_textbox_ax = plt.axes([0.65, 0.08, 0.1, 0.04])
    end_x_textbox = TextBox(end_x_textbox_ax, 'end_x ', initial='5')
    end_y_textbox_ax = plt.axes([0.65, 0.02, 0.1, 0.04])
    end_y_textbox = TextBox(end_y_textbox_ax, 'end_y ', initial='4')


    plt.show()




if __name__ == '__main__':
    l_1, l_2 = map(float, input('L1 L2: ').split())
    start_point = tuple(map(float, input(f'X1 Y1: ').split()))
    end_point = tuple(map(float, input(f'X2 Y2: ').split()))
    discretization = 100
    update_discretization(l_1, l_2, start_point, end_point, discretization)



