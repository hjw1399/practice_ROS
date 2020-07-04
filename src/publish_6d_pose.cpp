#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <random>
#include <iostream>

#define mean 0
#define std_dev 0.1

struct Pose6D
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_6d_pose");
  ros::Time::init();
  ros::NodeHandle n;
  ros::Publisher pub_sensor_raw_data = n.advertise<std_msgs::String>("vehicle_6d_pose", 1000);
  ros::Rate loop_rate(100);

  std::default_random_engine generator;
  double init_time = ros::Time::now().toSec();
  double past_time = ros::Time::now().toSec();
  double past_error(0);
  double integral_error(0);
  bool isFirstError = true;

  Pose6D pose6D = {0};
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    std::normal_distribution<double> distribution(mean, std_dev);

    Pose6D distribution6D;
    distribution6D.x = distribution(generator);
    distribution6D.y = distribution(generator);
    distribution6D.z = distribution(generator);
    distribution6D.roll = distribution(generator);
    distribution6D.pitch = distribution(generator);
    distribution6D.yaw = distribution(generator);

    pose6D.x += distribution6D.x;
    pose6D.y += distribution6D.y;
    pose6D.z += distribution6D.z;
    pose6D.roll += distribution6D.roll;
    pose6D.pitch += distribution6D.pitch;
    pose6D.yaw += distribution6D.yaw;

    if(ros::Time::now().toSec() - init_time >= 3.0)
    {
      double x_ref = 30;

      double Kp = 3;
      double Ki = 0.01;
      double Kd = 0.1;

      double dt = ros::Time::now().toSec() - past_time;

      double error = x_ref - pose6D.x;
      integral_error += error * dt;
      double dedt = (error - past_error)/dt;
      if(isFirstError)
      {
        dedt = 0;
        isFirstError = false;
      }

      past_error = error;
      double x_velocity = Kp * error + Ki * integral_error + Kd * dedt;
      pose6D.x += x_velocity * dt;
    }

    ss << "*" << pose6D.x << "," << pose6D.y << "," << pose6D.z << "," <<
                pose6D.roll << "," << pose6D.pitch << "," << pose6D.yaw;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    pub_sensor_raw_data.publish(msg);

    pose6D.x -= distribution6D.x;
    pose6D.y -= distribution6D.y;
    pose6D.z -= distribution6D.z;
    pose6D.roll -= distribution6D.roll;
    pose6D.pitch -= distribution6D.pitch;
    pose6D.yaw -= distribution6D.yaw;

    past_time = ros::Time::now().toSec();
    loop_rate.sleep();
  }

//  delete[] generator;
  return 0;
}