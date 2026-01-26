import math
import random
import matplotlib.pyplot as plt

dt = 0.01
track_width = 0.2
v_command = 0.1
omega_command = 0
steps = 500

#PID commands
Kp = 3
Kd = 0.5
Ki = 0.5

def wheel_speed(v, omega, track_width):
    v_left = v - (omega * track_width / 2)
    v_right = v + (omega * track_width / 2)
    return v_left, v_right

def step (x, y, theta, v_left, v_right, dt, track_width):
    v = (v_left + v_right) / 2
    omega = (v_right - v_left) / track_width
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt
    return x , y, theta

x, y = 0.0, 0.0
theta = 0.35 #radians

x_path, y_path = [], []
theta_no_control, theta_p_control, theta_pd_control, theta_pid_control  = [], [], [], []
errors_through_time = [random.uniform(-0.02, 0.02) for i in range(steps)]
time = [i * dt for i in range(steps)]

for i in range(steps):
    theta += errors_through_time[i]
    omega_command = 0
    v_left, v_right = wheel_speed(v_command, omega_command, track_width)
    x, y, theta = step(x, y, theta, v_left, v_right, dt, track_width)
    theta_no_control.append(theta)
    #theta_direction.append(theta)

x, y = 0.0, 0.0
theta = 0.35 #radians
previous_error = 0.0

for i in range(steps):
    error = 0 - theta
    theta += errors_through_time[i]
    omega_command = Kp * error
    v_left, v_right = wheel_speed(v_command, omega_command, track_width)
    x, y, theta = step(x, y, theta, v_left, v_right, dt, track_width)
    theta_p_control.append(theta)
    previous_error = error

x, y = 0.0, 0.0
theta = 0.35 #radians
previous_error = 0.0

for i in range(steps):
    error = 0 - theta
    theta += errors_through_time[i]
    error_rate = (error - previous_error) / dt
    omega_command = Kp * error + Kd * error_rate
    v_left, v_right = wheel_speed(v_command, omega_command, track_width)
    x, y, theta = step(x, y, theta, v_left, v_right, dt, track_width)
    theta_pd_control.append(theta)
    previous_error = error
    
x, y = 0.0, 0.0
theta = 0.35 #radians
previous_error = 0.0
I = 0.0

for i in range(steps):
    error = 0 - theta
    theta += errors_through_time[i]
    error_rate = (error - previous_error) / dt
    I += error * dt
    omega_command = Kp * error + Kd * error_rate + Ki * I
    
    v_left, v_right = wheel_speed(v_command, omega_command, track_width)
    x, y, theta = step(x, y, theta, v_left, v_right, dt, track_width)

    theta_pid_control.append(theta)
    previous_error = error

plt.figure()
plt.plot(time, theta_no_control, label="No Controller")
plt.plot(time, theta_p_control, label="P Controller")
plt.plot(time, theta_pd_control, label="PD Controller")
plt.plot(time, theta_pid_control, label="PID Controller")
plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.title("Theta vs Time: PD vs No Control")
plt.legend()
plt.savefig("images/robot_path.png")