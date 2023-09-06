import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

# Global variable for the animation
ani = None


def get_solution(l_1, l_2, l_3, gamma, target_point):
    solutions = []
    x, y = target_point
    l_vector = np.sqrt(l_2 ** 2 + l_3 ** 2 + 2 * l_2 * l_3 * np.cos(gamma))
    big_vector = np.sqrt(x ** 2 + y ** 2)

    cos_fi = ((l_vector ** 2 + l_1 ** 2 - big_vector ** 2) / (2 * l_vector * l_1))
    if cos_fi < -1 or cos_fi > 1:
        return []
    cos_beta = np.cos(np.pi - np.arccos(cos_fi))

    if cos_beta < -1 or cos_beta > 1:
        return []
    sin_beta = np.sqrt(1 - cos_beta ** 2)

    beta_1 = np.arctan2(sin_beta, cos_beta)
    beta_2 = np.arctan2(-sin_beta, cos_beta)

    # Calculate alpha_1 and alpha_2 using arccos and arctan2
    alpha_1 = np.arctan2(y, x) - np.arctan2(l_vector * np.sin(beta_1), l_1 + l_vector * np.cos(beta_1))
    alpha_2 = np.arctan2(y, x) - np.arctan2(l_vector * np.sin(beta_2), l_1 + l_vector * np.cos(beta_2))

    solutions.append((alpha_1, beta_1, gamma))
    solutions.append((alpha_2, beta_2, gamma))

    return solutions


def can_reach_target(l_1, l_2, l_3, target_point, base_point):
    """Check if a three-link manipulator can reach the target point."""
    distance_to_target = np.linalg.norm(np.array(target_point) - np.array(base_point))

    # Учитываем сумму всех длин l_1, l_2 и l_3
    if distance_to_target <= l_1 + l_2 + l_3:
        return True
    return False


def update_plot(frame, target_point, gamma_discretization, l_1, l_2, l_3, lines):
    """Update the plot for each frame."""
    angles = get_solution(l_1, l_2, l_3, gamma_discretization[frame], target_point)

    for i, (alpha, beta, gamma) in enumerate(angles):
        l_vector = np.sqrt(l_2 ** 2 + l_3 ** 2 + 2 * l_2 * l_3 * np.cos(gamma))
        beta_1 = np.arccos((l_2 ** 2 + l_vector ** 2 - l_3 ** 2) / (2 * l_vector * l_2))
        x1, y1 = l_1 * np.cos(alpha), l_1 * np.sin(alpha)
        x2, y2 = x1 + l_2 * np.cos(alpha + beta + beta_1), y1 + l_2 * np.sin(alpha + beta + beta_1)
        x3, y3 = l_1 * np.cos(alpha) + l_vector * np.cos(alpha + beta), l_1 * np.sin(alpha) + l_vector * np.sin(alpha + beta)

        lines[i].set_data([0, x1, x2, x3], [0, y1, y2, y3])
        print(np.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2))
    return lines


def update_speed(val):
    """Update animation speed based on slider value."""
    ani.event_source.stop()  # Stop the current animation
    ani.event_source.interval = 1000 / val  # Update the interval
    ani.event_source.start()  # Restart the animation with the new interval


def main():
    # Input parameters
    l_1, l_2, l_3 = 6, 1, 3  # map(float, input('L1 L2 L3: ').split())
    target_point = -4, 4  # tuple(map(float, input('X, Y:').split()))
    base_point = (0, 0)

    if not can_reach_target(l_1, l_2, l_3, target_point, base_point):
        print("Введите другие параметры")
        return

    fig, ax = plt.subplots()
    ax.set_xlim([-l_1 - l_2 - l_3, l_1 + l_2 + l_3])
    ax.set_ylim([-l_1 - l_2 - l_3, l_1 + l_2 + l_3])
    ax.set_title("2D Planar Manipulator")
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.grid(True)

    # Create lines for the manipulator's links
    lines = [ax.plot([], [], 'o-')[0] for _ in range(2)]

    # Add speed control slider
    ax_slider = plt.axes([0.2, 0.02, 0.6, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Speed', 1, 500, valinit=100)
    slider.on_changed(update_speed)

    # Alpha discretization
    gamma_discretization = list(i / 100 for i in range(0, 36000))

    # Animation
    global ani
    ani = FuncAnimation(fig, update_plot,
                        frames=len(gamma_discretization),
                        interval=1000 / slider.val,
                        fargs=(target_point, gamma_discretization, l_1, l_2, l_3, lines),
                        blit=True)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
