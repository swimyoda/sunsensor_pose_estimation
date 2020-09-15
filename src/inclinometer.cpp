#include <ros/ros.h>
#include "sunsensor_pose_estimation/eulAng.h"
#include <random>
#include <cmath>

class Inclinometer
{
public: 
	Inclinometer()
	{
		pub = nh.advertise<sunsensor_pose_estimation::eulAng>("/incl_data", 1);
  		true_sub = nh.subscribe("/ground_truth", 0.3, &Inclinometer::callbackTruth, this);
	}
private:
	double pitch, roll, yaw;
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber true_sub;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution;
	double g = 9.81; // m/s^2
	
	void callbackTruth(const sunsensor_pose_estimation::eulAngConstPtr & pose)
	{
		pitch = pose->pitch;
		roll  = pose->roll;
		//ROS_INFO("%f %f", pitch, roll);
		double f[3] = {	-sin(roll)      		+ distribution(generator)*0.000002,
						 cos(roll)*sin(pitch)	+ distribution(generator)*0.000002,
					  	-cos(pitch)*cos(roll)   + distribution(generator)*0.000002};
		roll  = asin(-f[0]);
		pitch = asin(f[1]/cos(roll));
		yaw   = 0;
		//ROS_INFO("%f %f", pitch, roll);
		send_msg();
	}

	sunsensor_pose_estimation::eulAng send_msg()
	{
	  sunsensor_pose_estimation::eulAng msg;
	  msg.roll   =  roll; 
	  msg.pitch  =  pitch;
	  msg.yaw    =  yaw;
	  pub.publish(msg);
	}
};

int main(int argc, char *argv[])
{
		ros::init(argc, argv, "inclinometer");
		Inclinometer inclinometer;
		ros::spin();
  
  
  

}
