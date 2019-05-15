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
  camera_info_sub_ = n_.subscribe(camera_info_topic_name, 1, &MilImageGroundTruth::CameraInfoCB, this);
  objects_pub_ = n_.advertise<mil_msgs::ObjectsInImage>("/image_ground_truth/"+(parentSensor->Name()), 1000);

  debug_pub_map_ = n_.advertise<geometry_msgs::PointStamped>("/DEBUG/"+(parentSensor->Name())+"/map", 1);
  debug_pub_cam_ = n_.advertise<geometry_msgs::PointStamped>("/DEBUG/"+(parentSensor->Name())+"/cam", 1);
}

/*
void MilImageGroundTruth::OnNewFrame(const unsigned char *_image,
                                     unsigned int _width, unsigned int _height,
                                     unsigned int _depth, const std::string &_format)
{
  object_in_image_.clear();
  gazebo::physics::Model_V models =gazebo::physics::get_world(parentSensor->WorldName())->GetModels(); 
  object_in_image_.resize(models.size());
  
  {
  auto i = object_in_image_.begin();
  for (auto j = models.begin(); j!=models.end(); ++j, ++i)
  {
    i->name = (*j)->GetName();
    ignition::math::Vector3d v
    (
      (*j)->GetWorldPose().pos.x,
      (*j)->GetWorldPose().pos.y,
      (*j)->GetWorldPose().pos.z
    );
    i->confidence = 1.0;
    i->points.resize(1);
    i->points.at(0).x = parentSensor->Camera()->Project(v)[0];
    i->points.at(0).y = parentSensor->Camera()->Project(v)[1];
    
  }
  }
  return;
}
*/

void MilImageGroundTruth::CameraInfoCB(const sensor_msgs::CameraInfoConstPtr& _cam_info)
{
  cam_info_=_cam_info;
}

void MilImageGroundTruth::ImageCB(const sensor_msgs::Image& _msg)
{
  camera_.fromCameraInfo(cam_info_);
  tf::StampedTransform tf;
  try
  {
    tf_listener_.waitForTransform(camera_.tfFrame(), "map" , _msg.header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform(camera_.tfFrame(), "map", _msg.header.stamp, tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }
  catch (tf::LookupException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }/*
  catch (tf::ExtrapolationException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }*/
  mil_msgs::ObjectsInImage objects_in_image;
  objects_in_image.header = _msg.header;
  
  for(gazebo::physics::ModelPtr i : gazebo::physics::get_world(parentSensor->WorldName())->GetModels()) 
  {
    mil_msgs::ObjectInImage object_in_image;
    object_in_image.confidence = 1.0;
    object_in_image.name = i->GetName();

    geometry_msgs::PointStamped p;
    p.header.stamp = _msg.header.stamp;
    p.header.frame_id = "map";

    p.point.x = i->GetWorldPose().pos.x;
    p.point.y = i->GetWorldPose().pos.y;
    p.point.z = i->GetWorldPose().pos.z;

    geometry_msgs::PointStamped pPrime;
    pPrime.header.stamp = _msg.header.stamp;
    pPrime.header.frame_id = camera_.tfFrame();
    
    tf_listener_.transformPoint(camera_.tfFrame(), p, pPrime);
    if (i->GetName() == "dice")
    {
      debug_pub_map_.publish(p);
      debug_pub_cam_.publish(pPrime);
    }
    cv::Point3d pPrimeCV;
    pPrimeCV.x = pPrime.point.x;
    pPrimeCV.y = pPrime.point.y;
    pPrimeCV.z = pPrime.point.z;

    cv::Point2d q = camera_.project3dToPixel(pPrimeCV);

    object_in_image.points.resize(1);
    object_in_image.points.at(0).x = q.x;
    object_in_image.points.at(0).y = q.y;


    objects_in_image.objects.push_back(object_in_image);
  }
   
  objects_pub_.publish(objects_in_image);

  return;
}
}
