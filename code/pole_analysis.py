import numpy as np
import matplotlib.pyplot as plt
from Pathfinder import run_pathfinder

def plot_heading_pid_poles(Kp, Ki, Kd):
    a = 1 + Kd
    b = Kp
    c = Ki

    poles = np.roots([a, b, c])

    plt.figure()
    plt.axhline(0)
    plt.axvline(0)
    plt.scatter(poles.real, poles.imag, marker='x')
    plt.xlabel("Real")
    plt.ylabel("Imag")
    plt.title("Heading PID Closed-Loop Poles")
    plt.grid(True)
    plt.savefig("images/Poles/Pole_analysis.png")
if __name__ == "__main__":
    run_pathfinder()
    plot_heading_pid_poles(Kp=3, Ki=0.1, Kd=0.7)
