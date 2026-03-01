#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/cuda/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

Eigen::Matrix3d extRot;
Eigen::Matrix3d extRPY;
Eigen::Vector3d extTrans;
Eigen::Quaterniond extQRPY;
struct ParamServer
{
  std::string robot_id;

  // Topics
  string pointCloudTopic;
  string imuTopic;
  string odomTopic;
  string gpsTopic;

  // Frames
  string lidarFrame;
  string baselinkFrame;
  string odometryFrame;
  string mapFrame;

  // GPS Settings
  bool useImuHeadingInitialization;
  bool useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // Save pcd
  bool savePCD;
  string savePCDDirectory;

  // Lidar Sensor Configuration
  SensorType sensor;
  int N_SCAN;
  int Horizon_SCAN;
  int downsampleRate;
  float lidarMinRange;
  float lidarMaxRange;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;
  float imuRPYWeight;
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int edgeFeatureMinValidNum;
  int surfFeatureMinValidNum;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool loopClosureEnableFlag;
  float loopClosureFrequency;
  int surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;
};

ParamServer loadParams(ros::NodeHandle& nh)
{
  ParamServer paramServer;
  ROS_WARN("here");
  // nh.param<std::string>("/robot_id", paramServer.robot_id, "roboat");

  nh.param<std::string>("lio_sam/pointCloudTopic", paramServer.pointCloudTopic, "points_raw");
  nh.param<std::string>("lio_sam/imuTopic", paramServer.imuTopic, "imu_correct");
  nh.param<std::string>("lio_sam/odomTopic", paramServer.odomTopic, "odometry/imu");
  nh.param<std::string>("lio_sam/gpsTopic", paramServer.gpsTopic, "odometry/gps");

  nh.param<std::string>("lio_sam/lidarFrame", paramServer.lidarFrame, "base_link");
  nh.param<std::string>("lio_sam/baselinkFrame", paramServer.baselinkFrame, "base_link");
  nh.param<std::string>("lio_sam/odometryFrame", paramServer.odometryFrame, "odom");
  nh.param<std::string>("lio_sam/mapFrame", paramServer.mapFrame, "map");

  nh.param<bool>("lio_sam/useImuHeadingInitialization", paramServer.useImuHeadingInitialization, false);
  nh.param<bool>("lio_sam/useGpsElevation", paramServer.useGpsElevation, false);
  nh.param<float>("lio_sam/gpsCovThreshold", paramServer.gpsCovThreshold, 2.0);
  nh.param<float>("lio_sam/poseCovThreshold", paramServer.poseCovThreshold, 25.0);

  nh.param<bool>("lio_sam/savePCD", paramServer.savePCD, false);
  nh.param<std::string>("lio_sam/savePCDDirectory", paramServer.savePCDDirectory, "/Downloads/LOAM/");
  ROS_WARN(" param server sett");
  std::string sensorStr;
  nh.param<std::string>("lio_sam/sensor", sensorStr, "");
  if (sensorStr == "velodyne")
  {
    paramServer.sensor = SensorType::VELODYNE;
  }
  else if (sensorStr == "ouster")
  {
    paramServer.sensor = SensorType::OUSTER;
  }
  else if (sensorStr == "livox")
  {
    paramServer.sensor = SensorType::LIVOX;
  }
  else
  {
    // ROS_ERROR_STREAM(
    //   "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox') setting to livox: " << sensorStr);
    sensorStr == "livox";
    // ros::shutdown();
  }

  nh.param<int>("lio_sam/N_SCAN", paramServer.N_SCAN, 16);
  nh.param<int>("lio_sam/Horizon_SCAN", paramServer.Horizon_SCAN, 1800);
  nh.param<int>("lio_sam/downsampleRate", paramServer.downsampleRate, 1);
  nh.param<float>("lio_sam/lidarMinRange", paramServer.lidarMinRange, 1.0);
  nh.param<float>("lio_sam/lidarMaxRange", paramServer.lidarMaxRange, 1000.0);

  nh.param<float>("lio_sam/imuAccNoise", paramServer.imuAccNoise, 0.01);
  nh.param<float>("lio_sam/imuGyrNoise", paramServer.imuGyrNoise, 0.001);
  nh.param<float>("lio_sam/imuAccBiasN", paramServer.imuAccBiasN, 0.0002);
  nh.param<float>("lio_sam/imuGyrBiasN", paramServer.imuGyrBiasN, 0.00003);
  nh.param<float>("lio_sam/imuGravity", paramServer.imuGravity, 9.80511);
  nh.param<float>("lio_sam/imuRPYWeight", paramServer.imuRPYWeight, 0.01);
  nh.param<vector<double> >("lio_sam/extrinsicRot", paramServer.extRotV, vector<double>());
  nh.param<vector<double> >("lio_sam/extrinsicRPY", paramServer.extRPYV, vector<double>());
  nh.param<vector<double> >("lio_sam/extrinsicTrans", paramServer.extTransV, vector<double>());
  ROS_WARN_STREAM(" ext " << paramServer.extTransV.size());
  nh.param<float>("lio_sam/edgeThreshold", paramServer.edgeThreshold, 0.1);
  nh.param<float>("lio_sam/surfThreshold", paramServer.surfThreshold, 0.1);
  nh.param<int>("lio_sam/edgeFeatureMinValidNum", paramServer.edgeFeatureMinValidNum, 10);
  nh.param<int>("lio_sam/surfFeatureMinValidNum", paramServer.surfFeatureMinValidNum, 100);

  nh.param<float>("lio_sam/odometrySurfLeafSize", paramServer.odometrySurfLeafSize, 0.2);
  nh.param<float>("lio_sam/mappingCornerLeafSize", paramServer.mappingCornerLeafSize, 0.2);
  nh.param<float>("lio_sam/mappingSurfLeafSize", paramServer.mappingSurfLeafSize, 0.2);

  nh.param<float>("lio_sam/z_tollerance", paramServer.z_tollerance, FLT_MAX);
  nh.param<float>("lio_sam/rotation_tollerance", paramServer.rotation_tollerance, FLT_MAX);

  nh.param<int>("lio_sam/numberOfCores", paramServer.numberOfCores, 4);
  nh.param<double>("lio_sam/mappingProcessInterval", paramServer.mappingProcessInterval, 0.01);

  nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", paramServer.surroundingkeyframeAddingDistThreshold, 1.0);
  nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", paramServer.surroundingkeyframeAddingAngleThreshold, 0.2);
  nh.param<float>("lio_sam/surroundingKeyframeDensity", paramServer.surroundingKeyframeDensity, 1.0);
  nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", paramServer.surroundingKeyframeSearchRadius, 50.0);

  nh.param<bool>("lio_sam/loopClosureEnableFlag", paramServer.loopClosureEnableFlag, false);
  nh.param<float>("lio_sam/loopClosureFrequency", paramServer.loopClosureFrequency, 1.0);
  nh.param<int>("lio_sam/surroundingKeyframeSize", paramServer.surroundingKeyframeSize, 50);
  nh.param<float>("lio_sam/historyKeyframeSearchRadius", paramServer.historyKeyframeSearchRadius, 10.0);
  nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", paramServer.historyKeyframeSearchTimeDiff, 30.0);
  nh.param<int>("lio_sam/historyKeyframeSearchNum", paramServer.historyKeyframeSearchNum, 25);
  nh.param<float>("lio_sam/historyKeyframeFitnessScore", paramServer.historyKeyframeFitnessScore, 0.3);

  nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", paramServer.globalMapVisualizationSearchRadius, 1e3);
  nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", paramServer.globalMapVisualizationSearchRadius, 10.0);
  nh.param<float>("lio_sam/globalMapVisualizationLeafSize", paramServer.globalMapVisualizationLeafSize, 1.0);

  extRot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(paramServer.extRotV.data());
  extRPY = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(paramServer.extRPYV.data());
  extTrans = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(paramServer.extTransV.data());
  extQRPY = Eigen::Quaterniond(extRPY).inverse();
  ROS_WARN(" nh param done");
  usleep(100);
  return paramServer;
}

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
{
  sensor_msgs::Imu imu_out = imu_in;
  // rotate acceleration
  Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
  acc = extRot * acc;
  imu_out.linear_acceleration.x = acc.x();
  imu_out.linear_acceleration.y = acc.y();
  imu_out.linear_acceleration.z = acc.z();
  // rotate gyroscope
  Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
  gyr = extRot * gyr;
  imu_out.angular_velocity.x = gyr.x();
  imu_out.angular_velocity.y = gyr.y();
  imu_out.angular_velocity.z = gyr.z();
  // rotate roll pitch yaw
  Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
  Eigen::Quaterniond q_final = q_from * extQRPY;
  imu_out.orientation.x = q_final.x();
  imu_out.orientation.y = q_final.y();
  imu_out.orientation.z = q_final.z();
  imu_out.orientation.w = q_final.w();

  if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
  {
    ROS_WARN_THROTTLE(5.0, "Invalid quaternion (near-zero norm), skipping IMU orientation. Waiting for valid data...");
    imu_out.orientation.x = 0.0;
    imu_out.orientation.y = 0.0;
    imu_out.orientation.z = 0.0;
    imu_out.orientation.w = 1.0;
  }

  return imu_out;
}

// sensor_msgs::PointCloud2 publishCloud(ros::Publisher* thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
// {
//   sensor_msgs::PointCloud2 tempCloud;
//   pcl::toROSMsg(*thisCloud, tempCloud);
//   tempCloud.header.stamp = thisStamp;
//   tempCloud.header.frame_id = thisFrame;
//   if (thisPub->getNumSubscribers() != 0)
//   {
//     thisPub->publish(tempCloud);
//   }
//   return tempCloud;
// }

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub.getNumSubscribers() != 0)
  {
    thisPub.publish(tempCloud);
  }
  return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
  return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu* thisImuMsg, T* angular_x, T* angular_y, T* angular_z)
{
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu* thisImuMsg, T* acc_x, T* acc_y, T* acc_z)
{
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu* thisImuMsg, T* rosRoll, T* rosPitch, T* rosYaw)
{
  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

inline float pointDistance(PointType p)
{
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline float pointDistance(PointType p1, PointType p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
