import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

# Global variable for the animation
ani = None


def get_solution(l_1, l_2, l_3, beta, target_point):
    solutions = []
    x, y = target_point
    l_vector = np.sqrt(l_1 ** 2 + l_2 ** 2 + 2 * l_1 * l_2 * np.cos(beta))
    big_vector = np.sqrt(x ** 2 + y ** 2)

    cos_fi = ((l_vector ** 2 + l_3 ** 2 - big_vector ** 2)/(2 * l_vector * l_3))
    cos_delta = ((l_2 ** 2 + l_vector ** 2 - l_1 ** 2) / (2 * l_2 * l_vector))
    cos_gamma = np.radians(np.degrees(cos_fi) - np.degrees(cos_delta))

    if cos_gamma < -1 or cos_gamma > 1:
        return []
    sin_gamma = np.sqrt(1 - cos_gamma ** 2)

    gamma_1 = np.arctan2(sin_gamma, cos_gamma)
    gamma_2 = np.arctan2(-sin_gamma, cos_gamma)
    b_1 = np.sqrt(big_vector ** 2 + l_1 ** 2 - 2 * l_1 * big_vector * np.cos(gamma_1))
    b_2 = np.sqrt(big_vector ** 2 + l_1 ** 2 - 2 * l_1 * big_vector * np.cos(gamma_2))

    alpha_1 = np.arctan2(y, x) + np.arccos((l_1 ** 2 + big_vector ** 2 - b_1 ** 2)/(2 * l_1 * big_vector))
    alpha_2 = np.arctan2(y, x) - np.arccos((l_1 ** 2 + big_vector ** 2 - b_2 ** 2)/(2 * l_1 * big_vector))

    solutions.append((alpha_1, beta, gamma_1))
    solutions.append((alpha_2, beta, gamma_2))

    return solutions


def can_reach_target(l_1, l_2, l_3, target_point, base_point):
    """Check if a three-link manipulator can reach the target point."""
    distance_to_target = np.linalg.norm(np.array(target_point) - np.array(base_point))

    # Учитываем сумму всех длин l_1, l_2 и l_3
    if distance_to_target <= l_1 + l_2 + l_3:
        return True
    return False


def update_plot(frame, target_point, beta_discretization, l_1, l_2, l_3, lines):
    """Update the plot for each frame."""
    angles = get_solution(l_1, l_2, l_3, beta_discretization[frame], target_point)

    for i, (alpha, beta, gamma) in enumerate(angles):
        x1, y1 = l_1 * np.cos(alpha), l_1 * np.sin(alpha)
        x2, y2 = x1 + l_2 * np.cos(beta), y1 + l_2 * np.sin(beta)
        x3, y3 = x2 + l_3 * np.cos(beta + gamma), y2 + l_3 * np.sin(beta + gamma)
        lines[i].set_data([0, x1, x2, x3], [0, y1, y2, y3])

    return lines


def update_speed(val):
    """Update animation speed based on slider value."""
    ani.event_source.stop()  # Stop the current animation
    ani.event_source.interval = 1000 / val  # Update the interval
    ani.event_source.start()  # Restart the animation with the new interval


def main():
    # Input parameters
    l_1, l_2, l_3 = 2, 10, 12  # map(float, input('L1 L2 L3: ').split())
    target_point = 10, 10  # tuple(map(float, input('X, Y:').split()))
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
    beta_discretization = list(i / 100 for i in range(0, 36000))

    # Animation
    global ani
    ani = FuncAnimation(fig, update_plot,
                        frames=len(beta_discretization),
                        interval=1000 / slider.val,
                        fargs=(target_point, beta_discretization, l_1, l_2, l_3, lines),
                        blit=True)

    # Show plot
    plt.show()


if __name__ == '__main__':
    main()
