#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "mapOptimization.hpp"
#include <ros/ros.h>

namespace lio_sam
{
class MapOptimizationNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override
  {
    NODELET_WARN("Initializing map optimization nodelet...");
    ros::NodeHandle nh = getNodeHandle();
    m_map_optimization = std::make_unique<mapOptimization>(nh);
    m_map_optimization->startThreads();
  }

private:
  std::unique_ptr<mapOptimization> m_map_optimization;
};
}
PLUGINLIB_EXPORT_CLASS(lio_sam::MapOptimizationNodelet, nodelet::Nodelet);
