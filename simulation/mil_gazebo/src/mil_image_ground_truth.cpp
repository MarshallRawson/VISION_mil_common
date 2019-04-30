#include<mil_gazebo/mil_image_ground_truth.hpp>
namespace mil_gazebo
{

void MilImageGroundTruth::OnNewFrame(const unsigned char *_image,
                                     unsigned int _width, unsigned int _height,
                                     unsigned int _depth, const std::string &_format)
{
  models_ = gazebo::physics::get_world(parentSensor->WorldName())->GetModels();
  {
  auto i=models_.begin();
  auto j=projections_.begin();
  for(;i!=models_.end(),j!=projections_.end();++i,++j)
  {
    ignition::math::Vector3d v
    (
      (*i)->GetWorldPose().pos.x,
      (*i)->GetWorldPose().pos.y,
      (*i)->GetWorldPose().pos.z
    );
    *j = parentSensor->Camera()->Project(v);
  }
  }

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ObjectsInImage objects_in_image();
  
  return;
}
}
