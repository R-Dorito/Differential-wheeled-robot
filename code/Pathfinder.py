import math
import random
import matplotlib.pyplot as plt
from datetime import datetime

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

Kp = 1.75
Ki = 0.3
Kd = 0.7

# ------------------------------------------------
class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, robot_thiccness=0.2, angular_velocity=0.2, dt=0.01, max_steps=20000):
        # Robot state
        self.x = x
        self.y = y
        self.theta = theta
        self.current_waypoint = 0
        # ------------------------------------------------

        # Declare Variables
        self.angular_velocity = angular_velocity
        self.robot_thiccness = robot_thiccness
        self.dt = dt
        self.max_steps = max_steps
        # ------------------------------------------------

        #Path to draw into graph
        self.x_path, self.y_path = [], []
        # ------------------------------------------------

        #Declare Tollerances/buffers
        self.pos_buffer = 0.02
        self.angle_buffer = 0.03
        # ------------------------------------------------

        #PID and friends
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        # ------------------------------------------------

        #errors
        self.error_prev = 0.0
        # ------------------------------------------------

    #------------------------------------------------

    #Differential wheeled robot - Wiki
    #taken from Wikipedia (Viki pedea)
    #V = omega * R and I dont know what R is~
    #R = V/omega
    #omega * (V/omega + thiccness/2)
    #thus the deravation 

    def wheel_speed(self, v, omega):
        v_left = v - (omega * self.robot_thiccness / 2)
        v_right = v + (omega * self.robot_thiccness / 2)
        return v_left, v_right
    # ------------------------------------------------

    #stepper
    def step(self, v_left, v_right):
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.robot_thiccness
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += omega * self.dt
        # return x, y, theta
    # ------------------------------------------------

    def setpoint_calc(self, x_waypoint, y_waypoint):
        #determine the difference
        dx = x_waypoint - self.x
        dy = y_waypoint - self.y

        distance = math.sqrt(dx**2 + dy**2)
        theta_desired = math.atan2(dy, dx)
        return distance, theta_desired

    def pid_controller(self, theta_desired):
        error = ((theta_desired - self.theta) + math.pi) % (2 * math.pi) - math.pi 
        #above is angle wrapping so it is +/- 180 degrees
        #[ −π , +π ]                 
        self.integral += error * self.dt                     
        derivative = (error - self.error_prev) / self.dt    
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  
        self.error_prev = error
        return output

    #Main Loop time
    def run_pathfinder(self, waypoints):
        for _ in range(self.max_steps):
            x_waypoint, y_waypoint = waypoints[self.current_waypoint]

            #its a bumy ride
            
            self.theta += random.uniform(-0.05, 0.05)

            distance, theta_desired = self.setpoint_calc(x_waypoint, y_waypoint)
            omega_controller = self.pid_controller(theta_desired)
            self.integral = max(min(self.integral, 1.0), -1.0) #clamped

            #slow down as it approches the waypoint
            V = min(self.angular_velocity, 1.5 * distance) 

            v_left, v_right = self.wheel_speed(V, omega_controller)
            self.step(v_left, v_right)

            self.x_path.append(self.x)
            self.y_path.append(self.y)

            #if distance is less than the tollerance and the angle (+/-) is also within tollerance
            if distance < self.pos_buffer and abs(self.error_prev) < self.angle_buffer:
                self.current_waypoint += 1
                if self.current_waypoint == len(waypoints):
                    print("Final waypoint reached")
                    break
    # ------------------------------------------------

    #Graph PLotting
    def plot_path(self, waypoints, Kp, Ki, Kd):
        #file stuff
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"images/Poles/Robot-Path-Kp{Kp}_Ki{Ki}_Kd{Kd}-{timestamp}.png"        
        
        plt.figure()
        plt.plot(self.x_path, self.y_path, label="Robot Path")

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
        plt.title(f"2D graphs - Kp: {Kp} Ki: {Ki} Kd: {Kd}")

        plt.savefig(filename) #saving the file as linux is weird about show()
    # ------------------------------------------------

if __name__ == "__main__":
    robot = Robot()
    robot.run_pathfinder(waypoints)
    robot.plot_path(waypoints, Kp, Ki, Kd)
