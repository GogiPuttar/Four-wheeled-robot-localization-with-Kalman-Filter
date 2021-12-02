# Four-wheeled-robot-localization-with-Kalman-Filter
These are the files for a simulated robot and its controller built using Webots. The robot has 4 driven wheels and two independent steering motors making it a non-holonomic system. This warrants the use of an Ackerman steering mechanism for accurate positioning and the localization is done using wheel odometry and inertial measurements. As a result of the simulated surface with variable friction, bounce and unevenness, the mapping needs to be strengthened using a Kalman filter.  

Given a **list of coordinates**, the robot will **autonomously drive itself to each coordinate in the list in that order and stop when it reaches the final point**. To try this out for yourself, all you need to do is install [Webots](https://cyberbotics.com/), download the "kf_1" folder and load this world into the program. The default robot controller should be "kf_controller_final".     

The robot can:
* Calculate it's current location at any given moment with the help of a Kalman filter
* Traverse to any coordinate in the plane provided the distance between any two consecutive coordinates is slightly larger than the length of the robot
* Check whether it has reached a point, iterate to the next waypoint, calculate odometry and inertial values and can be coded to move at different speeds

The robot can't:
* Detect or avoid obstacles in its path
* Move its wheels at an angular velocity higher than 10 rad/s

Further improvements might include:
* Obstacle detection/ avoidance and higher level path awareness and planning
* Not having to stop while changing the steering angle of its wheels

This project is GPL-3.0 Liscenced to Aditya Nair under GogiPuttar.
