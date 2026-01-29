import numpy as np
import matplotlib.pyplot as plt
from Pathfinder import Robot, waypoints, Kp, Ki, Kd
from datetime import datetime

robert = Robot()

def plot_heading_pid_poles(Kp, Ki, Kd):
    a = 1 + Kd
    b = Kp
    c = Ki

    poles = np.roots([a, b, c])
    timestamp = datetime.now().strftime("%Y%m%d_%H-%M-%S")

    plt.figure()
    plt.axhline(0)
    plt.axvline(0)
    plt.scatter(poles.real, poles.imag, marker='x')
    plt.xlabel("Real")
    plt.ylabel("Imag")
    plt.title("Heading PID Closed-Loop Poles")
    plt.grid(True)
    plt.savefig(f"images/Poles/Poles-Kp{Kp}_Ki{Ki}_Kd{Kd}-{timestamp}.png")

if __name__ == "__main__":
    robert.run_pathfinder(waypoints)
    plot_heading_pid_poles(Kp, Ki, Kd)
    
