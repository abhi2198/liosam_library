#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "imuPreintegration.hpp"
#include <ros/ros.h>

namespace lio_sam
{
class ImuPreintegrationNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override
  {
    NODELET_WARN("Initializing IMUPreintegration nodelet...");
    ros::NodeHandle nh = getMTNodeHandle();
    // tfFusion->initialize(nh);
    m_transform_fusion = std::make_unique<TransformFusion>(nh);
    m_imu_preintegration = std::make_unique<IMUPreintegration>(nh);
  }

private:
  std::unique_ptr<IMUPreintegration> m_imu_preintegration;
  std::unique_ptr<TransformFusion> m_transform_fusion;
};
}
PLUGINLIB_EXPORT_CLASS(lio_sam::ImuPreintegrationNodelet, nodelet::Nodelet);
