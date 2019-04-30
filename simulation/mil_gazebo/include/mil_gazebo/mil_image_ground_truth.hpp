#ifndef MIL_IMAGE_GROUND_TRUTH_HH
#define MIL_IMAGE_GROUND_TRUTH_HH

#include <string>
#include <ros/ros.h>
//#include <tf/transform_listener.h>

#include <gazebo/plugins/CameraPlugin.hh>
//#include <gazebo_plugins/gazebo_ros_camera_utils.h>
//#include <gazebo_plugins/gazebo_ros_camera.h>

#include <mil_msgs/ObjectsInImage.h>
#include <mil_msgs/ObjectInImage.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <vector>

namespace mil_gazebo
{
class MilImageGroundTruth : public gazebo::CameraPlugin
{
  private:
    ros::NodeHandle n_;
    ros::Publisher pub_; 
    gazebo::physics::Model_V   models_;
    std::vector<ignition::math::Vector2i> projections_;
  public: MilImageGroundTruth() : gazebo::CameraPlugin()
  {
    pub_ = n_.advertise<ObjectsInImage>("marshalls_cool_topic", 1000);
  }

  public: ~MilImageGroundTruth(){}

  public: 
    void Load(gazebo::sensors::SensorPtr _parent, 
              sdf::ElementPtr            _sdf) override
    { 
      gazebo::CameraPlugin::Load(_parent, 
                                 _sdf);
    }
  protected:
    void OnNewFrame(const unsigned char *_image,
                          unsigned int   _width, unsigned int       _height,
                          unsigned int   _depth, const std::string &_format);

};
GZ_REGISTER_SENSOR_PLUGIN(MilImageGroundTruth);
}
#endif
