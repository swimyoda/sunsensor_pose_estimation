#include <ros/ros.h>
#include <ros/package.h>
#include "../../cspice/include/SpiceUsr.h"
#include "sunsensor_pose_estimation/s_vec.h"

sunsensor_pose_estimation::s_vec generate_msg(const char * time);
const char *kernel; 	// name of the kernel (which allows access to the postional data)

// Parameters for the ephemeris calculation 
double lat = -1.5359; //rad (~88 deg South)
double lon = pi_c()/3.0; //rad (arbitrary)
#define 	targ 	"Sun"
#define 	obs  	"Moon"
#define 	ref   	"IAU_EARTH"
#define 	abcorr 	"CN+S"

double lt;				// light time (unused, but a necissary output for the spice function
double et;				// ephemeris time (used to calculate the position of the sun)
double S_me[3];			// sun vector in the moon me frame
double Sh_me[3];		// unit sun vector in the moon me frame
double Sh_nav[3];		// unit sun vector in the nav frame
double mag;				// magnitude of the vector (used for normalizing)
double M_nav_me[3][3]; 	// rotation from moon me to nav (East, North, Up) frame

int main(int argc, char *argv[])
{
  // furnish the spice metakernel (give our program access to the JPL data)
  std::string kpath = ros::package::getPath("sunsensor_pose_estimation")+"/spice_kernels/metakernel";
  kernel = kpath.c_str();
  furnsh_c(kernel);
  
  // initialize the ros node
  ros::init(argc, argv, "spice_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sunsensor_pose_estimation::s_vec>("/spice_data", 10);
  
  // sets me -> nav rotation
  eul2m_c(lon, -halfpi_c()+lat, pi_c(), 3, 2, 1, M_nav_me);
  
  // can relatively easily change how the time changes, right now it simulates a single time (i.e. 1 June at  5:30 pm)
  ros::Rate r(1);
  int day = 1;
  char dtime[20];
  while (ros::ok())
  {
    snprintf(dtime, 20, "2020-06-%02dT17:30:30",day); // changes the day if we uncomment line 52
    auto msg = generate_msg(dtime); 
    pub.publish(msg);
    ROS_INFO("msg: %s %s %s \n", std::to_string(msg.x).c_str(),std::to_string(msg.y).c_str(), std::to_string(msg.z).c_str());
	
	// Allows it to cycle through the month of june, now it stays on June 1
	// day++;
	if(day > 30)
		day = 1;
    r.sleep();
  }
}

sunsensor_pose_estimation::s_vec  generate_msg(const char * time)
{
	sunsensor_pose_estimation::s_vec msg;
	str2et_c(time, &et); 							 // gets the time in the right form
	spkpos_c(targ, et, ref, abcorr, obs, S_me, &lt); // gets the position of the sun's center relative to Earth
	unorm_c(S_me, Sh_me, &mag); 					 // normalizes the sun vector found in that last step

	// rotate sun vector into the nav frame and send it as a message
	mxv_c(M_nav_me, Sh_me, Sh_nav);
	msg.x = Sh_nav[0];
	msg.y = Sh_nav[1];
	msg.z = Sh_nav[2];

	return msg;
}

