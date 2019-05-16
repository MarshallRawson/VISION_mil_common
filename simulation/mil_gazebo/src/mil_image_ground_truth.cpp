#include<mil_gazebo/mil_image_ground_truth.hpp>
namespace mil_gazebo
{
void MilImageGroundTruth::Load(gazebo::sensors::SensorPtr _parent,
                               sdf::ElementPtr            _sdf)
{
  gazebo::CameraPlugin::Load(_parent,
                             _sdf);
  std::string image_topic_name = "image_raw";
  if (_sdf->HasElement("imageTopicName"))
    image_topic_name = _sdf->Get<std::string>("imageTopicName");
  std::string camera_info_topic_name = "camera_info";
  if (_sdf->HasElement("cameraInfoTopicName"))
    camera_info_topic_name = _sdf->Get<std::string>("cameraInfoTopicName");

  image_sub_ = n_.subscribe(image_topic_name, 1, &MilImageGroundTruth::ImageCB, this);
  objects_pub_ = n_.advertise<mil_msgs::ObjectsInImage>("/image_ground_truth/"+(parentSensor->Name()), 1000);

  debug_pub_map_ = n_.advertise<geometry_msgs::PointStamped>("/DEBUG/"+(parentSensor->Name())+"/map", 1);
  debug_pub_cam_ = n_.advertise<geometry_msgs::PointStamped>("/DEBUG/"+(parentSensor->Name())+"/cam", 1);
}


void MilImageGroundTruth::OnNewFrame(const unsigned char *_image,
                                     unsigned int _width, unsigned int _height,
                                     unsigned int _depth, const std::string &_format)
{
  mil_msgs::ObjectsInImage objects_in_image;
  objects_in_image.header.stamp = ros::Time::now();
  
  for(gazebo::physics::ModelPtr i : gazebo::physics::get_world(parentSensor->WorldName())->GetModels()) 
  {
    mil_msgs::ObjectInImage object_in_image;
    object_in_image.confidence = 1.0;
    object_in_image.name = i->GetName();
    ignition::math::Vector3d v
    (
      i->GetWorldPose().pos.x,
      i->GetWorldPose().pos.y,
      i->GetWorldPose().pos.z
    );
    object_in_image.points.resize(1);
    object_in_image.points.at(0).x = parentSensor->Camera()->Project(v)[0];
    object_in_image.points.at(0).y = parentSensor->Camera()->Project(v)[1];

    objects_in_image.objects.push_back(object_in_image);
  }
  buffer.push(objects_in_image);
  return;
}

void MilImageGroundTruth::ImageCB(const sensor_msgs::Image& _msg)
{
  if(buffer.size() == 0)
    return;
  while(buffer.front().header.stamp < _msg.header.stamp && buffer.size() > 1)
    buffer.pop();
  ROS_INFO("objects buffer size: %d", buffer.size());
  ROS_INFO("object- image: %f",buffer.front().header.stamp- _msg.header.stamp);
  buffer.front().header = _msg.header;
  objects_pub_.publish(buffer.front());
  return;
}
}
