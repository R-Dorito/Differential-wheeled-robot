<h1>Python Controls program for Differential wheeled robot</h1>
<p>Here I developed a differential wheeled robot with two seperately controlled wheels designed to take the robot to differnt points of interests called Waypoints. The intent of this robot design is to test and understand Control Systems concepts and implement them in a simulated 2D envirionment. The simulation includes a waypoint tracking method using PID error correction techniques to control the trajectory of the robot. 

With each iteration and adjustment, the robot will head towards the waypoint at a constant velocity and adjust its orientationby increasing or decreasing each wheels' angular velocity based on the feedback given. To simulate real world issues, the robot will experience a random angle deflection to simulate a bumpy terrain, and the task of the robot is to continue to reach the waypoint with the assistance of the error corrections.

Some results are shown below:</p>
<img src="https://github.com/R-Dorito/Python-test/blob/main/images/PID_Diagram.png?raw=true"/>
<figcaption>PID Closed Loop Diagram of program controls</figcaption>

<img src="https://github.com/R-Dorito/Python-test/blob/main/images/Tollerances.png"/>
<figcaption>Path of travel by robot, showing X and Y direction in a 2D view. Showing position location and location of each waypoint. I had designed it so that it would always loop back to the starting point. The bumpiness of the lines is due to the random(-0.02, 0.02) feature which simulates a bumpy ride. This run has been designed to show a large variation in bumpiness with a low PID error correction. </figcaption>

<img src="https://github.com/R-Dorito/Python-test/blob/main/images/PID%20Tests/260123%20Oscillation%20compare%20kp3%20kd%200.5%20ki%200.5.png"/>
<figcaption>Orientation of robot (theta) and the changes that come from the PID error corrections. The blue lines at the top shows the original direction if there was no error correction, and we can see it can go flying off with lots of up and down angles in an unstable manner. The introduction of the PID variables, all shown in different configurations, is shown on the lower section of the graph. We ideally want the angle of the robot to be oriented towards zero, which is based off the desired angle minus the actual angle of the robot. It is shown that the inclusion of all error correction types sufficiently dampens the errors and reduces the overall oscillation of the system, bringing the angle of the robot close to zero and settling at a steady state.</figcaption>

<h2>What's next?</h2>
<p>As this is a simulation only, I would like to include some more features in it to try out different things.</p>
<ul>
<li>Poles plotting to visualise the PID.</li>
<li>Create a predefined noise generated terrain which remains constant each time.</li>
<li>The ability to select waypoints after the main loop has been completed. At the moment, I am plotting these in a graph so it would be good to develop a live view of the model. The movements would ideally be interpolated.</li>
<li>Eventually create this model in real life so I can model it live. This would be ideally be created with an arduino and a DC motor wheel.</li>
</ul>
