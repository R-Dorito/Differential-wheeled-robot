import math
import random
import matplotlib.pyplot as plt

# Points of interest I want my lines to go through
waypoints = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.5, 0.5),
    (0.75, 0.75),
    (0.75, 0.20),
    (0.0, 0.0)
]
# ------------------------------------------------

#Differential wheeled robot - Wiki
#taken from Wikipedia (Viki pedea)
#V = omega * R and I dont know what R is~
#R = V/omega
#omega * (V/omega + thiccness/2)
#thus the deravation 

def wheel_speed(v, omega, robot_thiccness):
    v_left = v - (omega * robot_thiccness / 2)
    v_right = v + (omega * robot_thiccness / 2)
    return v_left, v_right
# ------------------------------------------------

#stepper
def step(x, y, theta, v_left, v_right, dt, robot_thiccness):
    v = (v_left + v_right) / 2
    omega = (v_right - v_left) / robot_thiccness
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt
    return x, y, theta
# ------------------------------------------------

def setpoint_calc(x, y, x_waypoint, y_waypoint):
    #determine the difference
    dx = x_waypoint - x
    dy = y_waypoint - y

    distance = math.sqrt(dx**2 + dy**2)
    theta_desired = math.atan2(dy, dx)
    return distance, theta_desired

def pid_controller(theta_desired, theta, Kp, Ki, Kd, prev_error, integral, dt):
    error = ((theta_desired - theta) + math.pi) % (2 * math.pi) - math.pi 
    #above is angle wrapping so it is +/- 180 degrees
    #[ −π , +π ]                 
    integral += error * dt                     
    derivative = (error - prev_error) / dt    
    output = Kp * error + Ki * integral + Kd * derivative  
    return output, error

# Declare Variables
x, y, theta = 0.0, 0.0, 0.0
angular_velocity = 0.2
robot_thiccness = 0.2
dt = 0.01
max_steps = 20000
current_waypoint = 0
# ------------------------------------------------

#Declare Tollerances/buffers
pos_buffer = 0.02
angle_buffer = 0.03
# ------------------------------------------------

#Path to draw into graph
x_path, y_path = [], []
# ------------------------------------------------

#PID and friends
Kp = 3
Ki = 0.1
Kd = 0.7
integral = 0.0
# ------------------------------------------------

#errors
error_prev = 0.0
# ------------------------------------------------


#Main Loop time
for _ in range(max_steps):
    x_waypoint, y_waypoint = waypoints[current_waypoint]

    #its a bumy ride
    theta += random.uniform(-0.05, 0.05)

    distance, theta_desired = setpoint_calc(x, y, x_waypoint, y_waypoint)
    omega_controller, heading_error = pid_controller(theta_desired, theta, Kp, Ki, Kd, error_prev, integral, dt)
    
    error_prev = heading_error

    #slow down as it approches the waypoint
    V = min(angular_velocity, 1.5 * distance) 

    v_left, v_right = wheel_speed(V, omega_controller, robot_thiccness)
    x, y, theta = step(x, y, theta, v_left, v_right, dt, robot_thiccness)

    x_path.append(x)
    y_path.append(y)

    #if distance is less than the tollerance and the angle (+/-) is also within tollerance
    if distance < pos_buffer and abs(heading_error) < angle_buffer:
        current_waypoint += 1
        if current_waypoint == len(waypoints):
            print("Final waypoint reached")
            break
# ------------------------------------------------

#Graph PLotting
plt.figure()
plt.plot(x_path, y_path, label="Robot Path")

plt.scatter(
    [wp[0] for wp in waypoints],
    [wp[1] for wp in waypoints],
    color="red",
    label="Waypoints"
)

plt.axis("equal")
plt.legend()
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Robot Map Path 2D")

plt.savefig("images/Robot_Path.png") #saving the file as linux is weird about show()
# ------------------------------------------------
