#include "utility.h"
#include "lio_sam/cloud_info.h"

namespace lio_sam
{
struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const& left, smoothness_t const& right)
  {
    return left.value < right.value;
  }
};

class FeatureExtraction
{
  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  lio_sam::cloud_info cloudInfo;
  std_msgs::Header cloudHeader;
  ParamServer paramServer;

  std::vector<smoothness_t> cloudSmoothness;
  float* cloudCurvature;
  int* cloudNeighborPicked;
  int* cloudLabel;
  ros::NodeHandle* nh_;
  bool rosEnabled_;
  std::unique_ptr<ParamServer> m_param_server;   // Use unique_ptr for dynamic allocation
  void initializationValue();
  void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn);
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
  void freeCloudInfoMemory();
  void publishFeatureCloud();

public:
  FeatureExtraction(ros::NodeHandle&);
  FeatureExtraction(const ParamServer& params);

  // Direct API (non-ROS)
  void processCloudDirect(lio_sam::cloud_info& cloudInfoInOut,
                          const pcl::PointCloud<PointType>::Ptr& extractedCloudIn,
                          pcl::PointCloud<PointType>::Ptr& cornerCloudOut,
                          pcl::PointCloud<PointType>::Ptr& surfaceCloudOut);
};
};
