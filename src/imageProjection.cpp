# include "imageProjection.hpp"

namespace lio_sam
{
ImageProjection::ImageProjection(ros::NodeHandle& mnh) : nh_(&mnh), rosEnabled_(true), paramServer(loadParams(mnh)),
  deskewFlag(0)
{
  subImu = nh_->subscribe<sensor_msgs::Imu>(paramServer.imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
  subOdom = nh_->subscribe<nav_msgs::Odometry>(paramServer.odomTopic + "_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
  subLaserCloud = nh_->subscribe<sensor_msgs::PointCloud2>(paramServer.pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
  pubExtractedCloud = nh_->advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
  pubLaserCloudInfo = nh_->advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);
  ImageProjection::allocateMemory();
  ImageProjection::resetParameters();
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

ImageProjection::ImageProjection(const ParamServer& params) : nh_(nullptr), rosEnabled_(false), paramServer(params),
  deskewFlag(0)
{
  ImageProjection::allocateMemory();
  ImageProjection::resetParameters();
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

void ImageProjection::allocateMemory()
{
  ROS_WARN(" in allocate mem");
  laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

  fullCloud.reset(new pcl::PointCloud<PointType>());
  extractedCloud.reset(new pcl::PointCloud<PointType>());

  fullCloud->points.resize(paramServer.N_SCAN * paramServer.Horizon_SCAN);

  cloudInfo.startRingIndex.assign(paramServer.N_SCAN, 0);
  cloudInfo.endRingIndex.assign(paramServer.N_SCAN, 0);

  cloudInfo.pointColInd.assign(paramServer.N_SCAN * paramServer.Horizon_SCAN, 0);
  cloudInfo.pointRange.assign(paramServer.N_SCAN * paramServer.Horizon_SCAN, 0);
  imuTime = new double[queueLength];
  imuRotX = new double[queueLength];
  imuRotY = new double[queueLength];
  imuRotZ = new double[queueLength];

  // ImageProjection::resetParameters();
}

void ImageProjection::resetParameters()
{
  laserCloudIn->clear();
  extractedCloud->clear();
  // reset range matrix for range image projection
  rangeMat = cv::Mat(paramServer.N_SCAN, paramServer.Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

  imuPointerCur = 0;
  firstPointFlag = true;
  odomDeskewFlag = false;

  for (int i = 0; i < queueLength; ++i)
  {
    imuTime[i] = 0;
    imuRotX[i] = 0;
    imuRotY[i] = 0;
    imuRotZ[i] = 0;
  }
}

ImageProjection::~ImageProjection()
{
  delete[] imuTime;
  delete[] imuRotX;
  delete[] imuRotY;
  delete[] imuRotZ;
}

void ImageProjection::imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
  sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

  std::lock_guard<std::mutex> lock1(imuLock);
  imuQueue.push_back(thisImu);
}

void ImageProjection::odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(*odometryMsg);
}

void ImageProjection::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  if (!cachePointCloud(laserCloudMsg))
  {
    return;
  }

  if (!deskewInfo())
  {
    return;
  }

  ImageProjection::projectPointCloud();

  ImageProjection::cloudExtraction();

  ImageProjection::publishClouds();

  ImageProjection::resetParameters();
}

bool ImageProjection::cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  // cache point cloud
  cloudQueue.push_back(*laserCloudMsg);

  if (cloudQueue.size() <= 2)
  {
    return false;
  }
  else
  {
    currentCloudMsg = cloudQueue.front();
    cloudQueue.pop_front();

    cloudHeader = currentCloudMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();
    timeScanNext = cloudQueue.front().header.stamp.toSec();
  }

  // convert cloud
  pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

  // check dense flag
  if (laserCloudIn->is_dense == false)
  {
    ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
    ros::shutdown();
  }

  // check point time
  if (deskewFlag == 0)
  {
    deskewFlag = -1;
    for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
    {
      if (currentCloudMsg.fields[i].name == "timestamp")
      {
        deskewFlag = 1;
        break;
      }
    }
    if (deskewFlag == -1)
    {
      ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }
  }

  return true;
}

bool ImageProjection::deskewInfo()
{
  std::lock_guard<std::mutex> lock1(imuLock);
  std::lock_guard<std::mutex> lock2(odoLock);

  // make sure IMU data available for the scan
  if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
  {
    ROS_DEBUG("Waiting for IMU data ...");
    return false;
  }

  ImageProjection::imuDeskewInfo();

  ImageProjection::odomDeskewInfo();

  return true;
}

void ImageProjection::imuDeskewInfo()
{
  cloudInfo.imuAvailable = false;

  while (!imuQueue.empty())
  {
    if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
    {
      imuQueue.pop_front();
    }
    else
    {
      break;
    }
  }

  if (imuQueue.empty())
  {
    return;
  }

  imuPointerCur = 0;

  for (int i = 0; i < (int)imuQueue.size(); ++i)
  {
    sensor_msgs::Imu thisImuMsg = imuQueue[i];
    double currentImuTime = thisImuMsg.header.stamp.toSec();

    // get roll, pitch, and yaw estimation for this scan
    if (currentImuTime <= timeScanCur)
    {
      imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
    }

    if (currentImuTime > timeScanNext + 0.01)
    {
      break;
    }

    if (imuPointerCur == 0)
    {
      imuRotX[0] = 0;
      imuRotY[0] = 0;
      imuRotZ[0] = 0;
      imuTime[0] = currentImuTime;
      ++imuPointerCur;
      continue;
    }

    // get angular velocity
    double angular_x, angular_y, angular_z;
    imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

    // integrate rotation
    double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
    imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
    imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
    imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
    imuTime[imuPointerCur] = currentImuTime;
    ++imuPointerCur;
  }

  --imuPointerCur;

  if (imuPointerCur <= 0)
  {
    return;
  }

  cloudInfo.imuAvailable = true;
}

void ImageProjection::odomDeskewInfo()
{
  cloudInfo.odomAvailable = false;

  while (!odomQueue.empty())
  {
    if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
    {
      odomQueue.pop_front();
    }
    else
    {
      break;
    }
  }

  if (odomQueue.empty())
  {
    return;
  }

  if (odomQueue.front().header.stamp.toSec() > timeScanCur)
  {
    return;
  }

  // get start odometry at the beinning of the scan
  nav_msgs::Odometry startOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i)
  {
    startOdomMsg = odomQueue[i];

    if (ROS_TIME(&startOdomMsg) < timeScanCur)
    {
      continue;
    }
    else
    {
      break;
    }
  }

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // Initial guess used in mapOptimization
  cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
  cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
  cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
  cloudInfo.initialGuessRoll  = roll;
  cloudInfo.initialGuessPitch = pitch;
  cloudInfo.initialGuessYaw   = yaw;

  cloudInfo.odomAvailable = true;

  // get end odometry at the end of the scan
  odomDeskewFlag = false;

  if (odomQueue.back().header.stamp.toSec() < timeScanNext)
  {
    return;
  }

  nav_msgs::Odometry endOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i)
  {
    endOdomMsg = odomQueue[i];

    if (ROS_TIME(&endOdomMsg) < timeScanNext)
    {
      continue;
    }
    else
    {
      break;
    }
  }

  if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
  {
    return;
  }

  Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

  float rollIncre, pitchIncre, yawIncre;
  pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

  odomDeskewFlag = true;
}

void ImageProjection::findRotation(double pointTime, float* rotXCur, float* rotYCur, float* rotZCur)
{
  *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

  int imuPointerFront = 0;
  while (imuPointerFront < imuPointerCur)
  {
    if (pointTime < imuTime[imuPointerFront])
    {
      break;
    }
    ++imuPointerFront;
  }

  if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
  {
    *rotXCur = imuRotX[imuPointerFront];
    *rotYCur = imuRotY[imuPointerFront];
    *rotZCur = imuRotZ[imuPointerFront];
  }
  else
  {
    int imuPointerBack = imuPointerFront - 1;
    double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
    *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
    *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
  }
}

void ImageProjection::findPosition(double relTime, float* posXCur, float* posYCur, float* posZCur)
{
  *posXCur = 0; *posYCur = 0; *posZCur = 0;

  if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
  {
    return;
  }

  float ratio = relTime / (timeScanNext - timeScanCur);

  *posXCur = ratio * odomIncreX;
  *posYCur = ratio * odomIncreY;
  *posZCur = ratio * odomIncreZ;
}

PointType ImageProjection::deskewPoint(PointType* point, double relTime)
{
  if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
  {
    return *point;
  }
  double pointTime = timeScanCur + relTime;

  float rotXCur, rotYCur, rotZCur;
  findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

  float posXCur, posYCur, posZCur;
  findPosition(relTime, &posXCur, &posYCur, &posZCur);

  if (firstPointFlag == true)
  {
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
    firstPointFlag = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
  Eigen::Affine3f transBt = transStartInverse * transFinal;

  PointType newPoint;
  newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
  newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
  newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
  newPoint.intensity = point->intensity;

  return newPoint;
}

void ImageProjection::projectPointCloud()
{
  float verticalAngle;

  int cloudSize = laserCloudIn->points.size();

  // calculate start angle,end engle
  double startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
  double endOri = -atan2(laserCloudIn->points[cloudSize - 1].y,
                         laserCloudIn->points[cloudSize - 1].x) +
                  2 * M_PI;
  if (endOri - startOri > 3 * M_PI)
  {
    endOri -= 2 * M_PI;
  }
  else if (endOri - startOri < M_PI)
  {
    endOri += 2 * M_PI;
  }
  bool halfPassed = false;

  // range image projection

  for (int i = 0; i < cloudSize; ++i)
  {
    PointType thisPoint;
    thisPoint.x = laserCloudIn->points[i].x;
    thisPoint.y = laserCloudIn->points[i].y;
    thisPoint.z = laserCloudIn->points[i].z;
    thisPoint.intensity = laserCloudIn->points[i].intensity;

    ////////////////////////////////////////////////////////////////////////////////
    // int rowIdn = laserCloudIn->points[i].ring;                         //
    //
    verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x +       // change
                                            thisPoint.y * thisPoint.y)) *     //
                    180 / M_PI;                                               //
    int rowIdn = (verticalAngle + 15) / 2.0;                                  //
    ///////////////////////////////////////////////////////////////////////////////

    if (rowIdn < 0 || rowIdn >= paramServer.N_SCAN)
    {
      continue;
    }

    if (rowIdn % paramServer.downsampleRate != 0)
    {
      continue;
    }

    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    static float ang_res_x = 360.0 / float(paramServer.Horizon_SCAN);
    int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + paramServer.Horizon_SCAN / 2;
    if (columnIdn >= paramServer.Horizon_SCAN)
    {
      columnIdn -= paramServer.Horizon_SCAN;
    }

    if (columnIdn < 0 || columnIdn >= paramServer.Horizon_SCAN)
    {
      continue;
    }

    float range = pointDistance(thisPoint);

    if (range < 1.0)
    {
      continue;
    }

    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
    {
      continue;
    }

    // calculate reltime
    double ori = -atan2(laserCloudIn->points[i].y, laserCloudIn->points[i].x);
    if (!halfPassed)
    {
      if (ori < startOri - M_PI / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > startOri + M_PI * 3 / 2)
      {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI)
      {
        halfPassed = true;
      }
    }
    else
    {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > endOri + M_PI / 2)
      {
        ori -= 2 * M_PI;
      }
    }
    double relTime = (ori - startOri) / (endOri - startOri) * 0.1;

    thisPoint = deskewPoint(&thisPoint, relTime);      // rslidar

    rangeMat.at<float>(rowIdn, columnIdn) = pointDistance(thisPoint);

    int index = columnIdn + rowIdn * paramServer.Horizon_SCAN;
    fullCloud->points[index] = thisPoint;
  }
}

void ImageProjection::cloudExtraction()
{
  int count = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < paramServer.N_SCAN; ++i)
  {
    cloudInfo.startRingIndex[i] = count - 1 + 5;

    for (int j = 0; j < paramServer.Horizon_SCAN; ++j)
    {
      if (rangeMat.at<float>(i, j) != FLT_MAX)
      {
        // mark the points' column index for marking occlusion later
        cloudInfo.pointColInd[count] = j;
        // save range info
        cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
        // save extracted cloud
        extractedCloud->push_back(fullCloud->points[j + i * paramServer.Horizon_SCAN]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.endRingIndex[i] = count - 1 - 5;
  }
}

void ImageProjection::publishClouds()
{
  if (rosEnabled_)
  {
    cloudInfo.header = cloudHeader;
    cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, paramServer.lidarFrame);
    pubLaserCloudInfo.publish(cloudInfo);
  }
}

// --- Direct API (non-ROS) ---

void ImageProjection::addImuDirect(double stamp, double ax, double ay, double az,
                                   double gx, double gy, double gz,
                                   double qx, double qy, double qz, double qw)
{
  sensor_msgs::Imu imuMsg;
  imuMsg.header.stamp = ros::Time(stamp);
  // Data is assumed to already be in the correct frame (already converted)
  imuMsg.linear_acceleration.x = ax;
  imuMsg.linear_acceleration.y = ay;
  imuMsg.linear_acceleration.z = az;
  imuMsg.angular_velocity.x = gx;
  imuMsg.angular_velocity.y = gy;
  imuMsg.angular_velocity.z = gz;
  imuMsg.orientation.x = qx;
  imuMsg.orientation.y = qy;
  imuMsg.orientation.z = qz;
  imuMsg.orientation.w = qw;

  std::lock_guard<std::mutex> lock1(imuLock);
  imuQueue.push_back(imuMsg);
}

void ImageProjection::addOdomDirect(double stamp, const Eigen::Affine3f& pose, int resetCount)
{
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(pose, x, y, z, roll, pitch, yaw);

  nav_msgs::Odometry odomMsg;
  odomMsg.header.stamp = ros::Time(stamp);
  odomMsg.pose.pose.position.x = x;
  odomMsg.pose.pose.position.y = y;
  odomMsg.pose.pose.position.z = z;
  odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  odomMsg.pose.covariance[0] = resetCount;

  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(odomMsg);
}

bool ImageProjection::processScanDirect(double stamp, double nextStamp,
                                        const pcl::PointCloud<PointXYZIRT>::Ptr& cloud,
                                        lio_sam::cloud_info& cloudInfoOut,
                                        pcl::PointCloud<PointType>::Ptr& extractedCloudOut)
{
  // Set timestamps directly (bypass cachePointCloud)
  timeScanCur = stamp;
  timeScanNext = nextStamp;

  // Copy cloud directly
  *laserCloudIn = *cloud;

  // Check if time field is available (enable deskewing if points have time)
  if (deskewFlag == 0)
  {
    // Assume time is available since we're using PointXYZIRT
    bool hasTime = false;
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      if (cloud->points[i].time != 0.0f)
      {
        hasTime = true;
        break;
      }
    }
    deskewFlag = hasTime ? 1 : -1;
  }

  // Run deskewing (uses imuQueue and odomQueue)
  if (!deskewInfo())
  {
    // If no IMU data, still process but without deskewing
    cloudInfo.imuAvailable = false;
    cloudInfo.odomAvailable = false;
  }

  // Run projection and extraction
  projectPointCloud();
  cloudExtraction();

  // Copy results
  cloudInfoOut = cloudInfo;
  extractedCloudOut.reset(new pcl::PointCloud<PointType>());
  *extractedCloudOut = *extractedCloud;

  // Reset for next call
  resetParameters();

  return (extractedCloudOut->size() > 0);
}

};
