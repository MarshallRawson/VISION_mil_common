#ifndef MIL_MAP2_HH
#define MIL_MAP2_HH


#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace mil_gazebo
{
class MilMap2 : public gazebo::WorldPlugin
{
  private:
    ros::NodeHandle n_;
    tf2_ros::StaticTransformBroadcaster map_to_map2_broadcaster_; 
    geometry_msgs::TransformStamped map_to_map2_;


  public: MilMap2() : WorldPlugin(){}
  
  public: ~MilMap2(){}

  public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf);
};
GZ_REGISTER_WORLD_PLUGIN(MilMap2)
}
#endif
