#include "LioSamCore.h"
#include "imageProjection.hpp"
#include "featureExtraction.hpp"
#include "mapOptimization.hpp"
#include "imuPreintegration.hpp"

namespace lio_sam
{

LioSamCore::LioSamCore(const ParamServer& config)
  : config_(config)
  , lastScanTime_(-1.0)
{
  imageProjection_.reset(new ImageProjection(config_));
  featureExtraction_.reset(new FeatureExtraction(config_));
  mapOpt_.reset(new mapOptimization(config_));
  imuPreint_.reset(new IMUPreintegration(config_));
}

LioSamCore::~LioSamCore()
{
}

void LioSamCore::reset()
{
  imageProjection_.reset(new ImageProjection(config_));
  featureExtraction_.reset(new FeatureExtraction(config_));
  mapOpt_.reset(new mapOptimization(config_));
  imuPreint_.reset(new IMUPreintegration(config_));
  lastScanTime_ = -1.0;
}

void LioSamCore::addImu(double stamp,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& gyro,
                        const Eigen::Quaterniond& orientation)
{
  // Feed IMU to ImageProjection (for deskewing)
  imageProjection_->addImuDirect(stamp,
                                 acc.x(), acc.y(), acc.z(),
                                 gyro.x(), gyro.y(), gyro.z(),
                                 orientation.x(), orientation.y(), orientation.z(), orientation.w());

  // Feed IMU to IMUPreintegration
  imuPreint_->addImuDirect(stamp,
                           acc.x(), acc.y(), acc.z(),
                           gyro.x(), gyro.y(), gyro.z());
}

bool LioSamCore::processScan(double stamp,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                             const std::vector<int>& rings,
                             const std::vector<float>& times,
                             Eigen::Affine3f& poseOut,
                             Eigen::MatrixXd& covarianceOut)
{
  if (cloud->empty())
  {
    return false;
  }

  // Convert PointXYZI + rings + times to PointXYZIRT
  pcl::PointCloud<PointXYZIRT>::Ptr cloudIRT(new pcl::PointCloud<PointXYZIRT>());
  cloudIRT->resize(cloud->size());
  cloudIRT->is_dense = true;

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    PointXYZIRT& pt = cloudIRT->points[i];
    pt.x = cloud->points[i].x;
    pt.y = cloud->points[i].y;
    pt.z = cloud->points[i].z;
    pt.intensity = cloud->points[i].intensity;
    pt.ring = (i < rings.size()) ? rings[i] : 0;
    pt.time = (i < times.size()) ? times[i] : 0.0f;
  }

  // Use a synthetic "next" stamp for deskewing (assume 100ms scan period)
  double nextStamp = stamp + 0.1;

  // If we have a previous scan, feed the IMU preintegration pose as odom
  // to ImageProjection for deskewing
  Eigen::Affine3f imuPose;
  if (imuPreint_->getLatestPose(imuPose))
  {
    static int resetCount = 0;
    imageProjection_->addOdomDirect(stamp, imuPose, resetCount);
  }

  // Step 1: Image Projection (deskewing + range image)
  lio_sam::cloud_info cloudInfoOut;
  pcl::PointCloud<PointType>::Ptr extractedCloud;
  bool projOk = imageProjection_->processScanDirect(stamp, nextStamp, cloudIRT, cloudInfoOut, extractedCloud);
  if (!projOk || !extractedCloud || extractedCloud->empty())
  {
    return false;
  }

  // Step 2: Feature Extraction
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;
  featureExtraction_->processCloudDirect(cloudInfoOut, extractedCloud, cornerCloud, surfaceCloud);

  if (!cornerCloud || !surfaceCloud || (cornerCloud->empty() && surfaceCloud->empty()))
  {
    return false;
  }

  // Step 3: Map Optimization
  bool isDegen = false;
  bool mapOk = mapOpt_->processFeaturesDirect(stamp, cloudInfoOut, cornerCloud, surfaceCloud,
                                              poseOut, covarianceOut, isDegen);

  if (!mapOk)
  {
    return false;
  }

  // Step 4: Feed pose back to IMU Preintegration for bias correction
  // Convert Affine3f to gtsam::Pose3
  Eigen::Matrix4f mat = poseOut.matrix();
  Eigen::Matrix4d matd = mat.cast<double>();
  gtsam::Pose3 lidarPose(gtsam::Rot3(matd.block<3,3>(0,0)),
                          gtsam::Point3(matd(0,3), matd(1,3), matd(2,3)));
  imuPreint_->processOdometryDirect(stamp, lidarPose, isDegen);

  lastScanTime_ = stamp;
  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LioSamCore::getLocalMap() const
{
  return mapOpt_->getLocalMap();
}

} // namespace lio_sam
