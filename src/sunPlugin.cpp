#include <ros/ros.h>
#include <ros/package.h>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/msgs/vector3d.pb.h"
#include "sunsensor_pose_estimation/s_vec.h"

namespace gazebo
{
	class sunPlugin : public WorldPlugin
	{

	public: void OnRosMsg(const sunsensor_pose_estimation::s_vec &_msg)
	{
		// Pull the data from the message and turn it into strings
	  	double x = _msg.x;
	  	double y = _msg.y;
	  	double z = _msg.z;
	  	std::string xs,ys,zs;
		xs = std::to_string(x);
	  	ys = std::to_string(y);
		zs = std::to_string(z);
		
		// An SDF that contains the direction we want the sun to point
		std::string data = "<?xml version='1.0' ?>\
				<sdf version='1.6'>\
					<!-- Light Source -->\
					<light type='directional' name='sun'>\
						<direction>" + xs + " " + ys + " " + zs + "</direction>\
						<cast_shadows>true</cast_shadows>\
					 </light>\
				</sdf>";
				
		// Create the SDF object, then turn it into a gazebo light message
	  	sdf::SDF sun;
	  	sun.SetFromString(data);
	  	sdf::ElementPtr light = sun.Root()->GetElement("light");
	  	msgs::Light lightMsg;
		lightMsg = gazebo::msgs::LightFromSDF(light);	
		
		// publish teh gazebo message		
	  	this->lightPub->Publish(lightMsg);
	}

	public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
	{
		if (!ros::isInitialized()) // initialize the ROS node if neccissary
		{
			int argc = 0;
			char** argv = NULL;
			ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
		}
		  
	  	// initiallize the gazebo node
	  	this->gzNode = transport::NodePtr(new transport::Node());
	  	this->gzNode->Init();
	  
		// create gazebo publisher
		this->lightPub = this->gzNode->Advertise<msgs::Light>("~/light/modify");
		
		// subscribe to the spice_data ros topic
		rosSub = nh.subscribe("/spice_data",5,&sunPlugin::OnRosMsg, this);	
	}

	// ROS stuff
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	private: ros::Subscriber rosSub;
	private: ros::NodeHandle nh;

	// gazebo stuff
	private: ignition::transport::Node node;
	private: transport::NodePtr gzNode;
	private: transport::PublisherPtr lightPub;
	
	};
	GZ_REGISTER_WORLD_PLUGIN(sunPlugin);
}  // namespace gazebo

