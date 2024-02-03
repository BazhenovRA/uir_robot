import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

ani = None


def get_path_points(start, end, discretization):
    """Compute linear path points between start and end."""
    x_start, y_start, z_start = start
    x_end, y_end, z_end = end
    x_values = np.linspace(x_start, x_end, discretization)
    y_values = np.linspace(y_start, y_end, discretization)
    z_values = np.linspace(z_start, z_end, discretization)
    return list(zip(x_values, y_values, z_values))


def get_angles(point, l_1, l_2, l_3):
    """Calculate joint angles for the manipulator to reach a point."""
    x, y, z = point
    beta_new = None
    lst = []
    beta_angles = list(i / 2 for i in range(0, 720))
    for i in range(len(beta_angles)):
        # beta_0 = beta_angles[i]
        beta = beta_angles[i]
        cos_alpha = (np.cos(beta) * x + np.sin(beta) * y) / (np.sqrt(x ** 2 + y ** 2))
        if -1 > cos_alpha or cos_alpha > 1:
            continue
        else:
            alpha = np.arccos(cos_alpha)
            lst.append((alpha, beta))
    kortezh = min(lst)
    beta_new = kortezh[1]
    z0 = 0
    x0, y0 = l_1 * np.cos(beta_new),  l_1 * np.sin(beta_new)
    x = x - x0
    y = y - y0
    z = z - z0

    cos_theta2 = (x ** 2 + y ** 2 + z ** 2 - l_2 ** 2 - l_3 ** 2) / (2 * l_2 * l_3)
    if cos_theta2 < -1 or cos_theta2 > 1:
        return []
    sin_theta2 = np.sqrt(1 - cos_theta2 ** 2)
    if np.isnan(sin_theta2):
        return []

    theta2_1 = np.arctan2(sin_theta2, cos_theta2)
    theta2_2 = np.arctan2(-sin_theta2, cos_theta2)

    theta1_1 = np.arctan2(z, np.sqrt(x ** 2 + y ** 2)) - np.arctan2(l_3 * np.sin(theta2_1), l_2 + l_3 * np.cos(theta2_1))
    theta1_2 = np.arctan2(z, np.sqrt(x ** 2 + y ** 2)) - np.arctan2(l_3 * np.sin(theta2_2), l_2 + l_3 * np.cos(theta2_2))

    return [(beta_new, theta1_1, theta2_1), (beta_new, theta1_2, theta2_2)]


def can_reach_target(l_1, l_2, l_3, target_point):
    """Check if a three-link manipulator can reach the target point."""
    x, y, z = target_point
    l_total = l_1 + l_2 + l_3

    distance_to_target = np.linalg.norm(np.array(target_point))
    if distance_to_target > l_total:
        return False

    distance_to_base = np.linalg.norm(np.array([x, y, 0]))
    if distance_to_base > l_1 + l_2:
        return False
    if z < 0 or z > l_3:
        return False

    return True


def update_plot(frame, path_points, l_1, l_2, l_3, lines):
    """Update the plot for each frame."""
    angles = get_angles(path_points[frame], l_1, l_2, l_3)

    for i, (beta, theta1, theta2) in enumerate(angles):
        x, y, z = l_1 * np.cos(beta), l_1 * np.sin(beta), 0
        x1, y1, z1 = x + l_2 * np.cos(theta1) * np.cos(beta), y + l_2 * np.cos(theta1) * np.sin(beta), l_2 * np.sin(theta1)
        x2, y2, z2 = x1 + l_3 * np.cos(theta1 + theta2) * np.cos(beta), y1 + l_3 * np.cos(theta1 + theta2) * np.sin(beta), z1 + l_3 * np.sin(theta1 + theta2)
        print(f'x2: {x2}, y2: {y2}, z2: {z2}')
        lines[i].set_data([0, x, x1, x2], [0, y, y1, y2])
        lines[i].set_3d_properties([0, z, z1, z2])

    return lines


def update_speed(val):
    """Update animation speed based on slider value."""
    ani.event_source.stop()  # Stop the current animation
    ani.event_source.interval = 1000 / val  # Update the interval
    ani.event_source.start()  # Restart the animation with the new interval


def main():
    # Input parameters
    l_1, l_2, l_3 = map(float, input('L1 L2 L3: ').split())
    start_point = tuple(map(float, input('X1 Y1 Z1: ').split()))
    end_point = tuple(map(float, input('X2 Y2 Z2: ').split()))

    if not can_reach_target(l_1, l_2, l_3, end_point):
        print("Введите другие параметры")
        return

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim([-l_1 - l_2 - l_3, l_1 + l_2 + l_3])
    ax.set_ylim([- l_1 - l_2 - l_3, l_1 + l_2 + l_3])
    ax.set_zlim([-l_1 - l_2 - l_3, l_1 + l_2 + l_3])
    ax.set_title("3D Planar Manipulator")
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.grid(True)

    # Create lines for the manipulator's links
    lines = [ax.plot([], [], [], 'o-')[0] for _ in range(3)]

    path_points = get_path_points(start_point, end_point, 1000)
    # Add speed control slider
    ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Speed', 1, 1000, valinit=450)
    slider.on_changed(update_speed)

    # Animation
    global ani
    ani = FuncAnimation(fig, update_plot,
                        frames=len(path_points),
                        interval=1000 / slider.val,
                        fargs=(path_points, l_1, l_2, l_3, lines),
                        blit=True)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
