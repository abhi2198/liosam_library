#pragma once
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "featureExtraction.hpp"
#include <ros/ros.h>

namespace lio_sam
{
class FeatureExtractionNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override
  {
    ROS_WARN("Initializing FeatureExtraction nodelet...");
    ros::NodeHandle p_nh = getNodeHandle();
    ROS_WARN("got nodehandle");
    m_feature_extraction = std::make_unique<FeatureExtraction>(p_nh);
  }

private:
  std::unique_ptr<FeatureExtraction> m_feature_extraction;
};
}
PLUGINLIB_EXPORT_CLASS(lio_sam::FeatureExtractionNodelet, nodelet::Nodelet);
