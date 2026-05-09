import numpy as np
import matplotlib.pyplot as plt
from lidar import Lidar

PORT_NAME = 'COM8' # changeable
BAUD_RATE = 230400 # protocol says that lidar has this baud rate
MAX_DISTANCE = 7000  # mm


def main():
    lidar = Lidar(PORT_NAME, BAUD_RATE)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    scatter = ax.scatter([], [], s=7, c='red', edgecolors='none')
    ax.scatter([0], [0], s=100, c='blue', marker='X', label='Lidar')

    # dead zone 150 mm
    dead_zone = plt.Circle((0, 0), 150, color='gray', fill=False, linestyle='--')
    ax.add_artist(dead_zone)

    ax.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
    ax.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    print("Scanning...")

    try:
        while True:
            lidar.process_data()
            distances = lidar.data[:, 0]
            angles = lidar.data[:, 1]

            mask = (distances > 10)
            dist = distances[mask]
            ang = angles[mask]

            if len(dist) > 0:
                radians = np.deg2rad(ang)
                x = dist * np.cos(radians)
                y = dist * np.sin(radians)
                scatter.set_offsets(np.c_[x, y])
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.01)

    except KeyboardInterrupt:
        print("User abort...")
    finally:
        lidar.close()

if __name__ == "__main__":
    main()