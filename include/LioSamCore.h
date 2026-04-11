#ifndef LIO_SAM_CORE_H_
#define LIO_SAM_CORE_H_

#include "utility.h"
#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Forward declarations
struct PointXYZIRT;

namespace lio_sam
{

class ImageProjection;
class FeatureExtraction;
class mapOptimization;
class IMUPreintegration;

class LioSamCore
{
public:
  LioSamCore(const ParamServer& config);
  ~LioSamCore();
  void reset();

  // Feed IMU measurement (call at IMU rate)
  // Data should already be rotated to the correct frame using extrinsics
  void addImu(double stamp,
              const Eigen::Vector3d& acc,
              const Eigen::Vector3d& gyro,
              const Eigen::Quaterniond& orientation);

  // Process a LiDAR scan
  // Returns true if a new pose was computed
  // If deskewedCloudOut is non-null, the deskewed/range-filtered cloud
  // produced by the image projection stage is returned in the lidar frame.
  // This is the same cloud that LIO-SAM uses internally for feature
  // extraction and mapping, so downstream consumers (e.g., loop closure)
  // can operate on the motion-compensated scan instead of the raw input.
  bool processScan(double stamp,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   const std::vector<int>& rings,
                   const std::vector<float>& times,
                   Eigen::Affine3f& poseOut,
                   Eigen::MatrixXd& covarianceOut,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr deskewedCloudOut = nullptr);

  // Get local scan map for visualization
  pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalMap() const;

private:
  ParamServer config_;
  std::unique_ptr<ImageProjection> imageProjection_;
  std::unique_ptr<FeatureExtraction> featureExtraction_;
  std::unique_ptr<mapOptimization> mapOpt_;
  std::unique_ptr<IMUPreintegration> imuPreint_;
  double lastScanTime_;
};

} // namespace lio_sam

#endif // LIO_SAM_CORE_H_
