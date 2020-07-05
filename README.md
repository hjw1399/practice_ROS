# practice_ROS
We will practice how to program Publisher and Subscriber by using ROS.

## publish_6d_pose.cpp decription
We may use the sensor to estimate vehicle's pose.
so, I made it as if I'm receiving an IMU sensor data by adding Gaussian distribution noise.
and I also wrote the PID position control by accelerating.

Try Making this std_msgs data to geometry_msgs, or sensor_msgs/Imu by using ros publisher and subscriber,
and visualize the 6D pose on the rviz.
