#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "imageProjection.hpp"
#include <ros/ros.h>

namespace lio_sam
{
class ImageProjectionNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override
  {
    ROS_WARN("Initializing Image Projection nodelet...");
    ros::NodeHandle nh = getNodeHandle();
    ROS_WARN("got nodehandle...");
    m_image_projection = std::make_unique<ImageProjection>(nh);
    ROS_WARN("complete");
  }

private:
  std::unique_ptr<ImageProjection> m_image_projection;   // FeatureExtraction is the node class
};
}
PLUGINLIB_EXPORT_CLASS(lio_sam::ImageProjectionNodelet, nodelet::Nodelet);
