#include <mil_gazebo/mil_map2.hpp>
namespace mil_gazebo
{
void MilMap2::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  tf2_ros::StaticTransformBroadcaster map_to_map2_broadcaster;
  geometry_msgs::TransformStamped map_to_map2;
  map_to_map2.header.stamp = ros::Time::now();
  map_to_map2.header.frame_id = "map";
  map_to_map2.child_frame_id = "map2";
  //where odometry thinks the sub starts every time gazbo is started  (these numbers are not perfect)
  map_to_map2.transform.translation.x = -0.193607642642;
  map_to_map2.transform.translation.y = -0.0875879583866;
  map_to_map2.transform.translation.z = 0.100750817837;

  map_to_map2.transform.rotation.x = 6.13057442662e-06;
  map_to_map2.transform.rotation.y = -0.00324607641174;
  map_to_map2.transform.rotation.z = 0.205285526297;
  map_to_map2.transform.rotation.w = 0.978696743452;
  map_to_map2_broadcaster.sendTransform(map_to_map2);
  ros::spin();
  return;
}
}
