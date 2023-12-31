import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

# Global variable for the animation
ani = None


def get_path_points(start, end, discretization):
    """Compute linear path points between start and end."""
    x_start, y_start = start
    x_end, y_end = end
    x_values = np.linspace(x_start, x_end, discretization)
    y_values = np.linspace(y_start, y_end, discretization)
    return list(zip(x_values, y_values))


def get_angles(point, l_1, l_2, base_point):
    """Calculate joint angles for the manipulator to reach a point."""
    x, y = point
    x0, y0 = base_point
    x = x - x0
    y = y - y0

    cos_theta2 = (x ** 2 + y ** 2 - l_1 ** 2 - l_2 ** 2) / (2 * l_1 * l_2)
    sin_theta2 = np.sqrt(1 - cos_theta2 ** 2)
    if np.isnan(sin_theta2):
        return []

    theta2_1 = np.arctan2(sin_theta2, cos_theta2)
    theta2_2 = np.arctan2(-sin_theta2, cos_theta2)

    theta1_1 = np.arctan2(y, x) - np.arctan2(l_2 * np.sin(theta2_1), l_1 + l_2 * np.cos(theta2_1))
    theta1_2 = np.arctan2(y, x) - np.arctan2(l_2 * np.sin(theta2_2), l_1 + l_2 * np.cos(theta2_2))

    return [(theta1_1, theta2_1), (theta1_2, theta2_2)]


def can_reach_target(l_1, l_2, start_point, end_point, base_point):
    """Check if a two-link manipulator can reach the target point."""
    distance_to_target = np.linalg.norm(np.array(end_point) - np.array(base_point))
    if distance_to_target < np.abs(l_1 - l_2) or distance_to_target > l_1 + l_2:
        return False
    return True


def update_plot(frame, path_points, l_1, l_2, lines, base_point):
    """Update the plot for each frame."""
    angles = get_angles(path_points[frame], l_1, l_2, base_point)
    x0, y0 = base_point
    for i, (theta1, theta2) in enumerate(angles):
        x1, y1 = x0 + l_1 * np.cos(theta1), y0 + l_1 * np.sin(theta1)
        x2, y2 = x1 + l_2 * np.cos(theta1 + theta2), y1 + l_2 * np.sin(theta1 + theta2)
        lines[i].set_data([x0, x1, x2], [y0, y1, y2])
    return lines


def update_speed(val):
    """Update animation speed based on slider value."""
    ani.event_source.stop()  # Stop the current animation
    ani.event_source.interval = 1000 / val  # Update the interval
    ani.event_source.start()  # Restart the animation with the new interval


def main():
    # Input parameters
    l_1, l_2 = map(float, input('L1 L2: ').split())
    start_point = tuple(map(float, input('X1 Y1: ').split()))
    end_point = tuple(map(float, input('X2 Y2: ').split()))
    base_point = tuple(map(float, input('X0 Y0: ').split()))
    if not can_reach_target(l_1, l_2, start_point, end_point, base_point):
        print("Введите другие параметры")
        return

    # Create plot
    x0, y0 = base_point
    fig, ax = plt.subplots()
    ax.set_xlim([-l_1 - l_2 + x0, l_1 + l_2 + x0])
    ax.set_ylim([-l_1 - l_2 + y0, l_1 + l_2 + y0])
    ax.set_title("2D Planar Manipulator")
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.grid(True)

    # Create lines for the manipulator's links
    lines = [ax.plot([], [], 'o-')[0] for _ in range(2)]

    # Create path points
    path_points = get_path_points(start_point, end_point, 1000)

    # Add speed control slider
    ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Speed', 1, 500, valinit=100)
    slider.on_changed(update_speed)

    # Animation
    global ani
    ani = FuncAnimation(fig, update_plot,
                        frames=len(path_points),
                        interval=1000 / slider.val,
                        fargs=(path_points, l_1, l_2, lines, base_point),
                        blit=True)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
