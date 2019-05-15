#ifndef MIL_IMAGE_GROUND_TRUTH_HH
#define MIL_IMAGE_GROUND_TRUTH_HH

#include <string>
#include <vector>

#include <ros/ros.h>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <mil_msgs/ObjectsInImage.h>
#include <mil_msgs/ObjectInImage.h>

namespace mil_gazebo
{
class MilImageGroundTruth : public gazebo::CameraPlugin
{
  private:
    ros::NodeHandle n_;
    ros::Publisher objects_pub_;

    ros::Publisher debug_pub_map_;
    ros::Publisher debug_pub_cam_;

    ros::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    sensor_msgs::CameraInfoConstPtr cam_info_;
    image_geometry::PinholeCameraModel camera_;
    tf::TransformListener tf_listener_;

  public: MilImageGroundTruth() : gazebo::CameraPlugin(), tf_listener_(ros::Duration(2.0)){}

  public: ~MilImageGroundTruth(){}

  public: 
    void Load(gazebo::sensors::SensorPtr _parent, 
              sdf::ElementPtr            _sdf);
/*  protected:
    void OnNewFrame(const unsigned char *_image,
                          unsigned int   _width, unsigned int       _height,
                          unsigned int   _depth, const std::string &_format);
*/

  protected:
    void CameraInfoCB(const sensor_msgs::CameraInfoConstPtr& _cam_info);
  protected:
    void ImageCB(const sensor_msgs::Image& msg);

};
GZ_REGISTER_SENSOR_PLUGIN(MilImageGroundTruth);
}
#endif
