#include "featureExtraction.hpp"

namespace lio_sam
{
FeatureExtraction::FeatureExtraction(ros::NodeHandle& mnh) : nh_(&mnh), rosEnabled_(true), paramServer(loadParams(mnh))
{
  ROS_WARN("FeatureExtraction initialized properly");
  subLaserCloudInfo = nh_->subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

  pubLaserCloudInfo = nh_->advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
  pubCornerPoints = nh_->advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
  pubSurfacePoints = nh_->advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);

  initializationValue();
}

FeatureExtraction::FeatureExtraction(const ParamServer& params) : nh_(nullptr), rosEnabled_(false), paramServer(params)
{
  initializationValue();
}

void FeatureExtraction::initializationValue()
{
  cloudSmoothness.resize(paramServer.N_SCAN * paramServer.Horizon_SCAN);

  downSizeFilter.setLeafSize(paramServer.odometrySurfLeafSize, paramServer.odometrySurfLeafSize, paramServer.odometrySurfLeafSize);

  extractedCloud.reset(new pcl::PointCloud<PointType>());
  cornerCloud.reset(new pcl::PointCloud<PointType>());
  surfaceCloud.reset(new pcl::PointCloud<PointType>());

  cloudCurvature = new float[paramServer.N_SCAN * paramServer.Horizon_SCAN];
  cloudNeighborPicked = new int[paramServer.N_SCAN * paramServer.Horizon_SCAN];
  cloudLabel = new int[paramServer.N_SCAN * paramServer.Horizon_SCAN];
}

void FeatureExtraction::laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
{
  cloudInfo = *msgIn;       // new cloud info
  cloudHeader = msgIn->header;       // new cloud header
  pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);       // new cloud for extraction

  FeatureExtraction::calculateSmoothness();

  FeatureExtraction::markOccludedPoints();

  FeatureExtraction::extractFeatures();

  FeatureExtraction::publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness()
{
  int cloudSize = extractedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++)
  {
    float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4]
                      + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2]
                      + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10
                      + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2]
                      + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4]
                      + cloudInfo.pointRange[i + 5];

    cloudCurvature[i] = diffRange * diffRange;      // diffX * diffX + diffY * diffY + diffZ * diffZ;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    // cloudSmoothness for sorting
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureExtraction::markOccludedPoints()
{
  int cloudSize = extractedCloud->points.size();
  // mark occluded points and parallel beam points
  for (int i = 5; i < cloudSize - 6; ++i)
  {
    // occluded points
    float depth1 = cloudInfo.pointRange[i];
    float depth2 = cloudInfo.pointRange[i + 1];
    int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

    if (columnDiff < 10)
    {
      // 10 pixel diff in range image
      if (depth1 - depth2 > 0.3)
      {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      }
      else if (depth2 - depth1 > 0.3)
      {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }
    // parallel beam
    float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
    float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

    if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
    {
      cloudNeighborPicked[i] = 1;
    }
  }
}

void FeatureExtraction::extractFeatures()
{
  cornerCloud->clear();
  surfaceCloud->clear();

  pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

  for (int i = 0; i < paramServer.N_SCAN; i++)
  {
    surfaceCloudScan->clear();

    for (int j = 0; j < 6; j++)
    {
      int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
      int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

      if (sp >= ep)
      {
        continue;
      }

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--)
      {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > paramServer.edgeThreshold)
        {
          largestPickedNum++;
          if (largestPickedNum <= 20)
          {
            cloudLabel[ind] = 1;
            cornerCloud->push_back(extractedCloud->points[ind]);
          }
          else
          {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
            {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
            {
              break;
            }
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < paramServer.surfThreshold)
        {
          cloudLabel[ind] = -1;
          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++)
          {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
            if (columnDiff > 10)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
            if (columnDiff > 10)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        if (cloudLabel[k] <= 0)
        {
          surfaceCloudScan->push_back(extractedCloud->points[k]);
        }
      }
    }

    surfaceCloudScanDS->clear();
    downSizeFilter.setInputCloud(surfaceCloudScan);
    downSizeFilter.filter(*surfaceCloudScanDS);

    *surfaceCloud += *surfaceCloudScanDS;
  }
}

void FeatureExtraction::freeCloudInfoMemory()
{
  cloudInfo.startRingIndex.clear();
  cloudInfo.endRingIndex.clear();
  cloudInfo.pointColInd.clear();
  cloudInfo.pointRange.clear();
}

void FeatureExtraction::publishFeatureCloud()
{
  if (rosEnabled_)
  {
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, paramServer.baselinkFrame);
    cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, paramServer.baselinkFrame);
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
  }
}

// --- Direct API (non-ROS) ---

void FeatureExtraction::processCloudDirect(lio_sam::cloud_info& cloudInfoInOut,
                                           const pcl::PointCloud<PointType>::Ptr& extractedCloudIn,
                                           pcl::PointCloud<PointType>::Ptr& cornerCloudOut,
                                           pcl::PointCloud<PointType>::Ptr& surfaceCloudOut)
{
  cloudInfo = cloudInfoInOut;
  *extractedCloud = *extractedCloudIn;

  calculateSmoothness();
  markOccludedPoints();
  extractFeatures();

  // Copy results
  cornerCloudOut.reset(new pcl::PointCloud<PointType>());
  surfaceCloudOut.reset(new pcl::PointCloud<PointType>());
  *cornerCloudOut = *cornerCloud;
  *surfaceCloudOut = *surfaceCloud;

  // Update cloudInfo (free memory of large fields not needed downstream)
  freeCloudInfoMemory();
  cloudInfoInOut = cloudInfo;
}

};
