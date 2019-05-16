#ifndef MIL_IMAGE_GROUND_TRUTH_HH
#define MIL_IMAGE_GROUND_TRUTH_HH

#include <string>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <mil_msgs/ObjectsInImage.h>
#include <mil_msgs/ObjectInImage.h>

namespace mil_gazebo
{
class MilImageGroundTruth : public gazebo::CameraPlugin
{
  private:
    ros::NodeHandle n_;
    ros::Publisher objects_pub_;
    ros::Subscriber image_sub_;

    ros::Publisher debug_pub_map_;
    ros::Publisher debug_pub_cam_;

    std::queue<mil_msgs::ObjectsInImage> buffer;

  public: MilImageGroundTruth() : gazebo::CameraPlugin(){}

  public: ~MilImageGroundTruth(){}

  public: 
    void Load(gazebo::sensors::SensorPtr _parent, 
              sdf::ElementPtr            _sdf);

  protected:
    void OnNewFrame(const unsigned char *_image,
                          unsigned int   _width, unsigned int       _height,
                          unsigned int   _depth, const std::string &_format);

  protected:
    void ImageCB(const sensor_msgs::Image& msg);

};
GZ_REGISTER_SENSOR_PLUGIN(MilImageGroundTruth);
}
#endif
