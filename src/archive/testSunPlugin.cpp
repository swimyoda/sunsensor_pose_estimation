#include <ros/ros.h>
#include <ros/package.h>
#include <s_vec_msg/s_vec.h>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/msgs/vector3d.pb.h"

namespace gazebo
{
  class sunPlugin : public WorldPlugin
  {

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      if (!ros::isInitialized())
	  {
	    ROS_FATAL_STREAM("Bruh");
	    return;
	  }
	  ROS_INFO("Good bruh");
	  
    }
    
    
	
  };
  GZ_REGISTER_WORLD_PLUGIN(sunPlugin);
}  // namespace gazebo

