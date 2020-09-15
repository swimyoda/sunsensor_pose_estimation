#include <ros/ros.h>
#include <random>
#include "../../cspice/include/SpiceUsr.h"
#include "sunsensor_pose_estimation/s_vec.h"
#include "sunsensor_pose_estimation/angles.h"
#include "sunsensor_pose_estimation/eulAng.h"

class SunSensor
{

public:
	SunSensor()
	{
		// constructor initiallizes our nodes
		pub_         = n_.advertise<sunsensor_pose_estimation::angles>("/ss_data", 0.5);
		spice_sub_   = n_.subscribe("/spice_data", 5, &SunSensor::callbackSpice, this);
		true_sub_    = n_.subscribe("/ground_truth", 0.3, &SunSensor::callbackTruth, this);
		
		// Static tranform from the sun sensor frame to the imu frame
		eul2m_c(-halfpi_c(), -pi_c()-halfpi_c()/3.0, 0, 1,2,3, R_ssl_imu); 
		eul2m_c(-halfpi_c(), -pi_c()+halfpi_c()/3.0, 0, 1,2,3, R_ssr_imu); 
	}
	
	void send_msg()
	{
		double Sh_ssl[3], Sh_ssr[3];
		// Normally distributed error with std of 0.1/3 degrees (from data sheet)
		double err[4] = {distribution(generator)*0.0333333*d2r,
						 distribution(generator)*0.0333333*d2r,
						 distribution(generator)*0.0333333*d2r,
						 distribution(generator)*0.0333333*d2r};
		
		// Transform Sh_nav_spice into the left and right sun sensor frames 
		mxv_c(R_ssl_nav,Sh_nav_spice, Sh_ssl);
		mxv_c(R_ssr_nav,Sh_nav_spice, Sh_ssr);
		
		// Generate the message with sun sensor measurements
		sunsensor_pose_estimation::angles msg;
		msg.alpha1 = asin(Sh_ssl[0]) + err[0]; //radians
		msg.beta1  = asin(Sh_ssl[1]) + err[1];
		msg.alpha2 = asin(Sh_ssr[0]) + err[2];
		msg.beta2  = asin(Sh_ssr[1]) + err[3];
		msg.err1   = 0; // error codes - we start assuming that there is no error, we check below
		msg.err2   = 0;
		
		// Sensors throw a type 11 error if either of the measurement angles would be out of the field of view (+/-60 degrees)
		// This is simulating the situation where "light detected, but not strong enough to be the sun"
		if (pow(msg.alpha1,2) > pow(pi_c()/3,2) || pow(msg.beta1,2) > pow(pi_c()/3,2) || Sh_ssl[2] < 0)
			msg.err1 = 11;
		if (pow(msg.alpha2,2) > pow(pi_c()/3,2)  || pow(msg.beta2,2) > pow(pi_c()/3,2) || Sh_ssr[2] < 0)
			msg.err2 = 11;
		pub_.publish(msg);
	}
	
	// receives the message from the spice ephemeris node
	void callbackSpice(const sunsensor_pose_estimation::s_vecConstPtr & svec)
	{
		// Pulls the sun vector from the spice ephemeris message
		Sh_nav_spice[0] = svec->x; 
		Sh_nav_spice[1] = svec->y; 
		Sh_nav_spice[2] = svec->z;
	}
	
	// takes the true pose for the rover
	void callbackTruth(const sunsensor_pose_estimation::eulAngConstPtr & pose)
	{
		// This can be changed based on the kind of message from the simulation
		// The important part is that it defines the transform from each sunsensor to the navigation frame
		eul2m_c(pose->roll, pose->pitch, pose->yaw, 1,2,3, R_imu_nav);
		mxm_c(R_ssl_imu,R_imu_nav,R_ssl_nav);
		mxm_c(R_ssr_imu,R_imu_nav,R_ssr_nav);
		send_msg();
	}
	
private:
	// ROS node stuff
	ros::NodeHandle n_;
	ros::Subscriber spice_sub_;
	ros::Subscriber true_sub_;
	ros::Publisher pub_;
	// Used to generate noise
	std::default_random_engine generator;
	std::normal_distribution<double> distribution;
	// For storing the ephemeris sun vector in the nav frame (received from spice publisher)
	double Sh_nav_spice[3];
	// Transformations: R_a_b transorms a vector from the b to the a frame
	double R_imu_nav[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double R_ssr_nav[3][3];
	double R_ssl_nav[3][3];
	double R_ssr_imu[3][3];
	double R_ssl_imu[3][3];
	// converts angle in degrees to radians
	double d2r = pi_c()/180;

}; // sunsensor class

int main(int argc, char *argv[])
{

    // initialize ros node
	ros::init(argc, argv, "sunsensor");
	SunSensor ssObj;
	ros::spin();
}








