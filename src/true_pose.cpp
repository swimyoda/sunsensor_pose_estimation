#include <ros/ros.h>
#include "sunsensor_pose_estimation/eulAng.h"

sunsensor_pose_estimation::eulAng generate_msg();
int a, b, c;


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "true_pose");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sunsensor_pose_estimation::eulAng>("/ground_truth", 1);
  ros::Rate r(1);
  a = -20; // starting angle (degrees) for roll
  c = -90; // starting angle (degrees) for pitch
  while (ros::ok())
  {
    auto msg = generate_msg();
    pub.publish(msg);
    r.sleep();
  }
}

sunsensor_pose_estimation::eulAng generate_msg()
{
  // ROS_INFO("%i %i %i", a, b, c); // prints the current pose
  sunsensor_pose_estimation::eulAng msg;
  
  // setting our euler angles in radians for our pose message
  msg.roll   =    (a)*M_PI/180; 
  msg.pitch  =    (0)*M_PI/180; // set to zero for this test
  msg.yaw    =    (c)*M_PI/180;
  
  // cycles through our simulated poses
  c++;
  if(c == 90)
  {
  	c = -90;
  	a++;a++;a++;a++;
  	if(a == 20)
  		a = 0;
  }
  return msg;
}
