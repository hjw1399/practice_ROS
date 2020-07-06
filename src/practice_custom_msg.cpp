#include "ros/ros.h"
#include "publish_6d_pose/Person.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <string>
#include <array>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "practice_custom_msg");
  ros::Time::init();
  ros::NodeHandle n;
  ros::Publisher pub_custom_msg = n.advertise<publish_6d_pose::Person>("person_state", 1000);

  publish_6d_pose::Person person;
  person.name = "JIWAN HAN";
  person.header.stamp = ros::Time::now();
  person.header.frame_id = "Human";
  person.age = 25;
  person.weight = 1;
  person.pose.pose.position.x = 0;
  person.pose.pose.position.y = 0;
  person.pose.pose.position.z = 0;
  person.pose.pose.orientation.x = 0;
  person.pose.pose.orientation.y = 0;
  person.pose.pose.orientation.z = 0;
  person.pose.pose.orientation.w = 1;
  boost::array<double, 36>  covariance_matrix{1,0,0,0,0,0,
                                              0,1,0,0,0,0,
                                              0,0,1,0,0,0,
                                              0,0,0,1,0,0,
                                              0,0,0,0,1,0,
                                              0,0,0,0,0,1};
  person.pose.covariance = covariance_matrix;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    pub_custom_msg.publish(person);
    loop_rate.sleep();
  }

  return 0;
}