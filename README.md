# Four-wheeled-robot-localization-with-Kalman-Filter
These are the files for a simulated robot and its controller built using Webots. The robot has 4 driven wheels and two independent steering motors making it a non-holonomic system. This warrants the use of an Ackerman steering mechanism for accurate positioning and the localization is done using wheel odometry and inertial measurements. As a result of the simulated surface with variable friction, bounce and unevenness, the mapping needs to be strengthened using a Kalman filter.  

Given a **list of coordinates**, the robot will **autonomously drive itself to each coordinate in the list in that order and stop when it reaches the final point**. To try this out for yourself, all you need to do is install [Webots](https://cyberbotics.com/), download the "kf_1" file and load the world into the program. The default robot controller should be "kf_controller_final".     

This project is GPL-3.0 Liscenced to Aditya Nair under GogiPuttar.
