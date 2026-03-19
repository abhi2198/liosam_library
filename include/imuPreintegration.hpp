#include "utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
namespace lio_sam
{
class TransformFusion
{
private:
  std::mutex mtx;

  ros::Subscriber subImuOdometry;
  ros::Subscriber subLaserOdometry;

  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  Eigen::Affine3f lidarOdomAffine;
  Eigen::Affine3f imuOdomAffineFront;
  Eigen::Affine3f imuOdomAffineBack;

  tf::TransformListener tfListener;
  tf::StampedTransform lidar2Baselink;
  ParamServer paramServer;
  ros::NodeHandle* nh_;
  bool rosEnabled_;
  double lidarOdomTime = -1;
  deque<nav_msgs::Odometry> imuOdomQueue;

  Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);
  void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
public:
  TransformFusion(ros::NodeHandle& nh);
};

class IMUPreintegration
{
private:
  std::mutex mtx;

  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher pubImuOdometry;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
  gtsam::Vector noiseModelBetweenBias;

  gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;

  std::deque<sensor_msgs::Imu> imuQueOpt;
  std::deque<sensor_msgs::Imu> imuQueImu;

  gtsam::Pose3 prevPose_;
  gtsam::Vector3 prevVel_;
  gtsam::NavState prevState_;
  gtsam::imuBias::ConstantBias prevBias_;

  gtsam::NavState prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool doneFirstOpt = false;
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2 optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values graphValues;
  ParamServer paramServer;
  const double delta_t = 0;

  int key = 1;
  ros::NodeHandle* nh_;
  bool rosEnabled_;
  // T_bl: tramsform points from lidar frame to imu frame
  gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
  // T_lb: tramsform points from imu frame to lidar frame
  gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

  void resetOptimization();
  void resetParams();
  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
  bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw);

public:
  IMUPreintegration(ros::NodeHandle& nh);
  IMUPreintegration(const ParamServer& params);

  // Direct API (non-ROS)
  void addImuDirect(double stamp, double ax, double ay, double az,
                    double gx, double gy, double gz);
  void processOdometryDirect(double stamp, const gtsam::Pose3& lidarPose,
                             bool isDegenerate);
  bool getLatestPose(Eigen::Affine3f& poseOut) const;
};
}
