#include "utility.h"
#include <yaml-cpp/yaml.h>

ParamServer loadParamsFromYaml(const std::string& yamlPath)
{
  ParamServer ps;
  YAML::Node config = YAML::LoadFile(yamlPath);

  // Navigate into lio_sam namespace if present
  YAML::Node n = config["lio_sam"] ? config["lio_sam"] : config;

  // Topics
  if(n["pointCloudTopic"]) ps.pointCloudTopic = n["pointCloudTopic"].as<std::string>();
  if(n["imuTopic"])        ps.imuTopic = n["imuTopic"].as<std::string>();
  if(n["odomTopic"])       ps.odomTopic = n["odomTopic"].as<std::string>();
  if(n["gpsTopic"])        ps.gpsTopic = n["gpsTopic"].as<std::string>();

  // Frames
  if(n["lidarFrame"])     ps.lidarFrame = n["lidarFrame"].as<std::string>();
  if(n["baselinkFrame"])  ps.baselinkFrame = n["baselinkFrame"].as<std::string>();
  if(n["odometryFrame"])  ps.odometryFrame = n["odometryFrame"].as<std::string>();
  if(n["mapFrame"])       ps.mapFrame = n["mapFrame"].as<std::string>();

  // GPS
  if(n["useImuHeadingInitialization"]) ps.useImuHeadingInitialization = n["useImuHeadingInitialization"].as<bool>();
  if(n["useGpsElevation"])  ps.useGpsElevation = n["useGpsElevation"].as<bool>();
  if(n["gpsCovThreshold"])  ps.gpsCovThreshold = n["gpsCovThreshold"].as<float>();
  if(n["poseCovThreshold"]) ps.poseCovThreshold = n["poseCovThreshold"].as<float>();

  // Save PCD
  if(n["savePCD"])          ps.savePCD = n["savePCD"].as<bool>();
  if(n["savePCDDirectory"]) ps.savePCDDirectory = n["savePCDDirectory"].as<std::string>();

  // Sensor
  if(n["sensor"])
  {
    std::string sensorStr = n["sensor"].as<std::string>();
    if(sensorStr == "velodyne")      ps.sensor = SensorType::VELODYNE;
    else if(sensorStr == "ouster")   ps.sensor = SensorType::OUSTER;
    else                             ps.sensor = SensorType::LIVOX;
  }
  if(n["N_SCAN"])          ps.N_SCAN = n["N_SCAN"].as<int>();
  if(n["Horizon_SCAN"])    ps.Horizon_SCAN = n["Horizon_SCAN"].as<int>();
  if(n["downsampleRate"])  ps.downsampleRate = n["downsampleRate"].as<int>();
  if(n["lidarMinRange"])   ps.lidarMinRange = n["lidarMinRange"].as<float>();
  if(n["lidarMaxRange"])   ps.lidarMaxRange = n["lidarMaxRange"].as<float>();

  // IMU
  if(n["imuAccNoise"])  ps.imuAccNoise = n["imuAccNoise"].as<float>();
  if(n["imuGyrNoise"])  ps.imuGyrNoise = n["imuGyrNoise"].as<float>();
  if(n["imuAccBiasN"])  ps.imuAccBiasN = n["imuAccBiasN"].as<float>();
  if(n["imuGyrBiasN"])  ps.imuGyrBiasN = n["imuGyrBiasN"].as<float>();
  if(n["imuGravity"])   ps.imuGravity = n["imuGravity"].as<float>();
  if(n["imuRPYWeight"]) ps.imuRPYWeight = n["imuRPYWeight"].as<float>();
  if(n["extrinsicRot"])   ps.extRotV = n["extrinsicRot"].as<std::vector<double>>();
  if(n["extrinsicRPY"])   ps.extRPYV = n["extrinsicRPY"].as<std::vector<double>>();
  if(n["extrinsicTrans"]) ps.extTransV = n["extrinsicTrans"].as<std::vector<double>>();

  // LOAM
  if(n["edgeThreshold"])           ps.edgeThreshold = n["edgeThreshold"].as<float>();
  if(n["surfThreshold"])           ps.surfThreshold = n["surfThreshold"].as<float>();
  if(n["edgeFeatureMinValidNum"])  ps.edgeFeatureMinValidNum = n["edgeFeatureMinValidNum"].as<int>();
  if(n["surfFeatureMinValidNum"])  ps.surfFeatureMinValidNum = n["surfFeatureMinValidNum"].as<int>();

  // Voxel filter
  if(n["odometrySurfLeafSize"])   ps.odometrySurfLeafSize = n["odometrySurfLeafSize"].as<float>();
  if(n["mappingCornerLeafSize"])  ps.mappingCornerLeafSize = n["mappingCornerLeafSize"].as<float>();
  if(n["mappingSurfLeafSize"])    ps.mappingSurfLeafSize = n["mappingSurfLeafSize"].as<float>();

  if(n["z_tollerance"])        ps.z_tollerance = n["z_tollerance"].as<float>();
  if(n["rotation_tollerance"]) ps.rotation_tollerance = n["rotation_tollerance"].as<float>();

  // CPU
  if(n["numberOfCores"])          ps.numberOfCores = n["numberOfCores"].as<int>();
  if(n["mappingProcessInterval"]) ps.mappingProcessInterval = n["mappingProcessInterval"].as<double>();

  // Surrounding map
  if(n["surroundingkeyframeAddingDistThreshold"])  ps.surroundingkeyframeAddingDistThreshold = n["surroundingkeyframeAddingDistThreshold"].as<float>();
  if(n["surroundingkeyframeAddingAngleThreshold"]) ps.surroundingkeyframeAddingAngleThreshold = n["surroundingkeyframeAddingAngleThreshold"].as<float>();
  if(n["surroundingKeyframeDensity"])              ps.surroundingKeyframeDensity = n["surroundingKeyframeDensity"].as<float>();
  if(n["surroundingKeyframeSearchRadius"])         ps.surroundingKeyframeSearchRadius = n["surroundingKeyframeSearchRadius"].as<float>();

  // Loop closure
  if(n["loopClosureEnableFlag"])         ps.loopClosureEnableFlag = n["loopClosureEnableFlag"].as<bool>();
  if(n["loopClosureFrequency"])          ps.loopClosureFrequency = n["loopClosureFrequency"].as<float>();
  if(n["surroundingKeyframeSize"])       ps.surroundingKeyframeSize = n["surroundingKeyframeSize"].as<int>();
  if(n["historyKeyframeSearchRadius"])   ps.historyKeyframeSearchRadius = n["historyKeyframeSearchRadius"].as<float>();
  if(n["historyKeyframeSearchTimeDiff"]) ps.historyKeyframeSearchTimeDiff = n["historyKeyframeSearchTimeDiff"].as<float>();
  if(n["historyKeyframeSearchNum"])      ps.historyKeyframeSearchNum = n["historyKeyframeSearchNum"].as<int>();
  if(n["historyKeyframeFitnessScore"])   ps.historyKeyframeFitnessScore = n["historyKeyframeFitnessScore"].as<float>();

  // Visualization
  if(n["globalMapVisualizationSearchRadius"]) ps.globalMapVisualizationSearchRadius = n["globalMapVisualizationSearchRadius"].as<float>();
  if(n["globalMapVisualizationPoseDensity"])  ps.globalMapVisualizationPoseDensity = n["globalMapVisualizationPoseDensity"].as<float>();
  if(n["globalMapVisualizationLeafSize"])     ps.globalMapVisualizationLeafSize = n["globalMapVisualizationLeafSize"].as<float>();

  // Set global extrinsics
  if(ps.extRotV.size() == 9 && ps.extTransV.size() == 3)
  {
    extRot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(ps.extRotV.data());
    extRPY = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(ps.extRPYV.data());
    extTrans = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(ps.extTransV.data());
    extQRPY = Eigen::Quaterniond(extRPY).inverse();
  }

  return ps;
}
