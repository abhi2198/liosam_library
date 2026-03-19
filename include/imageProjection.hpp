#include "utility.h"
#include "lio_sam/cloud_info.h"
#include "tf_conversions/tf_eigen.h"

struct PointXYZIRT
{
  PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y) (float, z, z) (float, intensity, intensity)
                                    (uint16_t, ring, ring) (float, time, time)
                                  )

const int queueLength = 500;

namespace lio_sam
{
class ImageProjection
{
private:
  std::mutex imuLock;
  std::mutex odoLock;

  ros::Subscriber subLaserCloud;
  ros::Publisher pubLaserCloud;

  ros::Publisher pubExtractedCloud;
  ros::Publisher pubLaserCloudInfo;

  ros::Subscriber subImu;
  std::deque<sensor_msgs::Imu> imuQueue;

  ros::Subscriber subOdom;
  std::deque<nav_msgs::Odometry> odomQueue;

  std::deque<sensor_msgs::PointCloud2> cloudQueue;
  sensor_msgs::PointCloud2 currentCloudMsg;

  double* imuTime;
  double* imuRotX;
  double* imuRotY;
  double* imuRotZ;

  int imuPointerCur;
  bool firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr extractedCloud;
  ParamServer paramServer;
  int deskewFlag;
  cv::Mat rangeMat;

  bool odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  lio_sam::cloud_info cloudInfo;
  double timeScanCur;
  double timeScanNext;
  std_msgs::Header cloudHeader;
  ros::NodeHandle* nh_;
  bool rosEnabled_;

  void allocateMemory();
  void resetParameters();

  // Callbacks
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);
  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);
  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  bool deskewInfo();
  void imuDeskewInfo();
  void odomDeskewInfo();
  void findRotation(double pointTime, float* rotXCur, float* rotYCur, float* rotZCur);
  void findPosition(double relTime, float* posXCur, float* posYCur, float* posZCur);
  PointType deskewPoint(PointType* point, double relTime);
  void projectPointCloud();
  void cloudExtraction();
  void publishClouds();

public:
  ImageProjection(ros::NodeHandle& nh);
  ImageProjection(const ParamServer& params);
  ~ImageProjection();

  // Direct API (non-ROS)
  void addImuDirect(double stamp, double ax, double ay, double az,
                    double gx, double gy, double gz,
                    double qx, double qy, double qz, double qw);
  void addOdomDirect(double stamp, const Eigen::Affine3f& pose, int resetCount);
  bool processScanDirect(double stamp, double nextStamp,
                         const pcl::PointCloud<PointXYZIRT>::Ptr& cloud,
                         lio_sam::cloud_info& cloudInfoOut,
                         pcl::PointCloud<PointType>::Ptr& extractedCloudOut);
};
}
