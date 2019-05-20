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

  image_sub_ = n_.subscribe(image_topic_name, 1, &MilImageGroundTruth::ImageCB, this);
  
  objects_pub_ = n_.advertise<mil_msgs::ObjectsInImage>("/image_ground_truth/"+(parentSensor->Name()), 1000);
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
    //if the projection puts it outside of image, dont publish it
    if (((object_in_image.points.at(0).x > -1)&&(object_in_image.points.at(0).x < width))&&((object_in_image.points.at(0).y > -1)&&(object_in_image.points.at(0).y < height)))
      objects_in_image.objects.push_back(object_in_image);
  }
  //putting the object in images in a buffer so that covance may be applied to the image publishing timing
  buffer.push(objects_in_image);
  return;
}

void MilImageGroundTruth::ImageCB(const sensor_msgs::Image& _msg)
{
  if(buffer.size() == 0)
    return;
  while(buffer.front().header.stamp < _msg.header.stamp && buffer.size() > 1)
    buffer.pop();
  buffer.front().header = _msg.header;
  objects_pub_.publish(buffer.front());
  return;
}
}
