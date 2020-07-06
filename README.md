# practice_ROS
We will practice how to program Publisher and Subscriber by using ROS.

## publish_6d_pose.cpp decription
We may use the sensor to estimate vehicle's pose.
so, I made it as if I'm receiving an IMU sensor data by adding Gaussian distribution noise.
and I also wrote the PID position control by accelerating. I added it because I thought it would be fun.

Subscribe to this virtual Imu data(std_msgs/String) and try publishing it using Sensor_msgs/Imu by using the ros publisher and subscriber.
And Try publishing by using geometry_msgs, tf.
and visualize the 6D pose on the rviz.
