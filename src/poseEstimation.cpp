#include <ros/ros.h>
#include <ros/package.h>
#include "../../cspice/include/SpiceUsr.h"
#include "sunsensor_pose_estimation/eulAng.h"
#include "sunsensor_pose_estimation/s_vec.h"
#include "sunsensor_pose_estimation/angles.h"

class PoseEst
{

public:
	PoseEst()
	{
		// constructor initiallizes our nodes
		pub_         = n_.advertise<sunsensor_pose_estimation::eulAng>("/ss_pose", 1);
		spice_sub_   = n_.subscribe("/spice_data", 5, &PoseEst::callbackOne, this);
		ss_sub_      = n_.subscribe("/ss_data"   , 0.1, &PoseEst::callback, this);
		incl_sub_    = n_.subscribe("/incl_data" , 0.9, &PoseEst::callbackTwo, this);
		true_sub_    = n_.subscribe("/ground_truth" , 0.35, &PoseEst::callbackThree, this);
		
		// set transformations from sun sensor to imu frames
		eul2m_c(0, pi_c()+halfpi_c()/3.0, halfpi_c(), 3,2,1, R_imu_ssl); 
		eul2m_c(0, pi_c()-halfpi_c()/3.0, halfpi_c(), 3,2,1, R_imu_ssr);
	}
	
	void callback(const sunsensor_pose_estimation::anglesConstPtr & angles)
	{
		sunsensor_pose_estimation::eulAng yawMsg;
		
		// if neither sensor can see the sun
		if(angles->err1==11&&angles->err2==11)
		{
			// holder for an indeterminate siuation
			yawMsg.yaw = -999;
		}
		else
		{
			// if the left sun sensor doesn't have an error, we calculate its sun vector
			if(angles->err1 == 0)
			{
			  Sh_SSl[0] = sin(angles->alpha1);
			  Sh_SSl[1] = sin(angles->beta1); 
			  Sh_SSl[2] = sqrt(1 - pow(Sh_SSl[0],2) - pow(Sh_SSl[1],2));
			  if(pow(Sh_SSl[0],2) + pow(Sh_SSl[1],2) > 1)
			  	Sh_SSl[2] = 0;
			  mxv_c(R_imu_ssl, Sh_SSl, Sh_IMU_ssl);
			}
			// if the right sun sensor doesn't have an error, we calulate its sun vector
			if(angles->err2 == 0) // our simulated "out of range" error for the right sensor
			{
			  Sh_SSr[0] = sin(angles->alpha2); 
			  Sh_SSr[1] = sin(angles->beta2); 
			  Sh_SSr[2] = sqrt(1 - pow(Sh_SSr[0],2) - pow(Sh_SSr[1],2));
			  if(pow(Sh_SSr[0],2) + pow(Sh_SSr[1],2) > 1)
			  	Sh_SSr[2] = 0;
			  mxv_c(R_imu_ssr, Sh_SSr, Sh_IMU_ssr);
			}
			// combines the two measurements into one Sh_IMU_ss
			combineSensors(Sh_IMU_ssl, Sh_IMU_ssr, angles->err1, angles->err2); // sets Sh_IMU_ss
			mxv_c(R_imuf_imu, Sh_IMU_ss, S_ssf);
			double Yaw_ss    = atan2(S_ssf[0],S_ssf[1])/d2r; 				// azimuth of the measured sun vector in the imuf frame
			double Yaw_spice = atan2(Sh_nav_spice[0],Sh_nav_spice[1])/d2r;	// azimuth of the calculated sun vector in the nav frame
			
			// here, we set both azimuths to be positive in the range [0,360)
			if (Yaw_spice < 0)
				Yaw_spice = 360 + Yaw_spice;
		    if (Yaw_ss < 0)
		    	Yaw_ss = 360 + Yaw_ss;
		    	
		    // heading determination
			estYaw = Yaw_ss - Yaw_spice;
			// determining error from simulated pose
			yawMsg.yaw = abs(estYaw-trueYaw); 
			yawMsg.pitch = 0; yawMsg.roll = 0; // we only care about predicting yaw here
			//ROS_INFO("Yaw Error: %f", abs(estYaw-trueYaw));
			
		}
		pub_.publish(yawMsg);
	}
	
	// receives ephemeris sun vector information
	void callbackOne(const sunsensor_pose_estimation::s_vecConstPtr & svec)
	{
		Sh_nav_spice[0] = svec->x; 
		Sh_nav_spice[1] = svec->y; 
		Sh_nav_spice[2] = svec->z;
	}
	
	// receives inclinometer pose angles
	void callbackTwo(const sunsensor_pose_estimation::eulAngConstPtr & pose)
	{
		eul2m_c(0, -pose->pitch, -pose->roll, 3,2,1, R_imuf_imu);
	}
	
	// reveives true pose information
	void callbackThree(const sunsensor_pose_estimation::eulAngConstPtr & pose)
	{
		trueYaw = pose->yaw/d2r;
		// we want the yaw to be positive in the range [0,360)
		if (trueYaw < 0)
				trueYaw = 360 + trueYaw;
	}
	
private:
	double d2r = pi_c()/180; 	// degrees to radians conversion
	double Sh_IMU_ss[3];	 	// sun vector in the IMU frame, from both measurements
	double Sh_nav_spice[3];		// sun vector calculated from the spice ephemeris data in the nav frame
	double R_imuf_imu[3][3];	// rotation from imu, to imu flat frame
	double R_imu_ssl[3][3], R_imu_ssr[3][3];  // rotations from each sun sensor frame to the imu frame
    double Sh_IMU_ssl[3] = {0.0, 0.0, 0.0}, Sh_IMU_ssr[3] = {0.0, 0.0, 0.0}; // sun vectors in the imu frame measured by the sun sensors
	double Sh_SSl[3], Sh_SSr[3];  		      // Sun vectors in the sun sensor frames
	double S_spice[3], S_ss[3], S_ssf[3];
	double trueYaw;				// the true heading of the rover
	double estYaw;				// the heading estimated by the algorithm
	
	// spice publishers and subscribers
	ros::NodeHandle n_;
	ros::Subscriber spice_sub_;
	ros::Subscriber ss_sub_;
	ros::Subscriber incl_sub_;
	ros::Subscriber true_sub_;
	ros::Publisher pub_;
	
	// gives us our final Sh_IMU_ss based on the sun vectors in the IMU frame measured by the two sun sensors
	void combineSensors(double l[3], double r[3], int err1, int err2)
	{
		bool lZ  = err1 == 11; // if the left sun sensor could not see the sun
		bool rZ  = err2 == 11; // if the right sun sensor could not see the sun
		if(lZ && rZ)
		{
		  // we can't calculate, sensor out of view
		}
		if(lZ || rZ) // if either the left or right sensor can't see the sun
		{
		  if(!rZ) // if only the right sun sensor can see the sun
		  {
		  	   Sh_IMU_ss[0] = r[0]; Sh_IMU_ss[1] = r[1]; Sh_IMU_ss[2] = r[2];
		  	   
		  }else // if only the left sun sensor can see the sun
		  {
		  	   Sh_IMU_ss[0] = l[0]; Sh_IMU_ss[1] = l[1]; Sh_IMU_ss[2] = l[2];
		  }

		}else // if both sensors can see the sun
		{
			// for now, we'll just average the two measurements
	  		Sh_IMU_ss[0] = 	(l[0] + r[0])/2;
			Sh_IMU_ss[1] = 	(l[1] + r[1])/2;
			Sh_IMU_ss[2] = 	(l[2] + r[2])/2;
		}
	}
}; // end PoseEst class



int main(int argc, char *argv[])
{
	// initialize node
	ros::init(argc, argv, "pose_est");
	PoseEst estimatorObj;
	ros::spin();
}


