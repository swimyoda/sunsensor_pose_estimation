#include <ros/ros.h>
#include "ss_msg/angles.h"

ss_msg::angles generate_msg();

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sunsensor");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<ss_msg::angles>("/ss_data", 1);

  ros::Rate r(1);
  while (ros::ok())
  {
    ROS_INFO("Publishing message...");
    auto msg = generate_msg();
    pub.publish(msg);
    ROS_INFO("Message has been published\n");

    r.sleep();
  }
}

ss_msg::angles generate_msg()
{
  ss_msg::angles msg;
  msg.alpha1 =  0.2792;
  msg.beta1  = -0.4015;
  msg.alpha2 = -100;
  msg.beta2  = -100;

  return msg;
}
