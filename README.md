# LIO-SAM Library

A refactored version of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) (Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping) by Tixiao Shan, originally published at IROS 2020. This fork restructures the codebase into a standalone C++ library (`LioSamCore`) that can be consumed by external projects without a running ROS system, while also supporting ROS1 nodelet-based deployment.

## Key Changes from Upstream LIO-SAM

### Nodelet Architecture

The original LIO-SAM runs each processing stage as a separate ROS node communicating via topics. This fork converts all stages into **nodelets** that run in a shared process, eliminating serialization/deserialization overhead between stages:

- `image_projection_nodelet` — Range image projection and scan deskewing
- `feature_extraction_nodelet` — Edge and surface feature extraction
- `map_optimization_nodelet` — Scan-to-map registration via GTSAM factor graph
- `imu_preintegration_nodelet` — IMU preintegration and bias estimation

Nodelets are defined in `nodelet_plugins.xml` and can be loaded into a single nodelet manager via the provided launch files.

### Non-ROS Library API (`LioSamCore`)

A standalone library (`liblio_sam_core.so`) exposes LIO-SAM's full odometry pipeline through a simple C++ interface that does not require `ros::NodeHandle`, topic subscriptions, or any ROS runtime:

```cpp
#include <LioSamCore.h>

// Configure via ParamServer struct
ParamServer config;
config.sensor = SensorType::VELODYNE;
config.N_SCAN = 16;
config.Horizon_SCAN = 1800;
// ... set IMU noise, feature thresholds, etc.

lio_sam::LioSamCore core(config);

// Feed IMU at IMU rate
core.addImu(stamp, acc, gyro, orientation);

// Process lidar scans — returns pose when ready
Eigen::Affine3f pose;
Eigen::MatrixXd covariance;
bool ok = core.processScan(stamp, cloud, rings, times, pose, covariance);
```

The pipeline internally runs all four stages (image projection, feature extraction, map optimization, IMU preintegration) in sequence on each scan, managing state across calls. This makes it straightforward to embed LIO-SAM as an odometry backend inside other SLAM frameworks or offline processing tools.

Each original ROS component has a dual constructor:
- `Component(ros::NodeHandle& nh)` — for nodelet/ROS usage, subscribes to topics
- `Component(const ParamServer& config)` — for library usage, no ROS dependencies at runtime

Direct-feed methods (`addImuDirect`, `processScanDirect`, `processFeaturesDirect`, etc.) bypass ROS message callbacks and accept raw data structures directly.

### YAML Config File Loader

A `loadParamsFromYaml()` function allows loading the full `ParamServer` configuration from a standard LIO-SAM YAML config file without a ROS parameter server:

```cpp
#include <utility.h>

ParamServer config = loadParamsFromYaml("/path/to/params.yaml");
lio_sam::LioSamCore core(config);
```

The loader handles the `lio_sam:` namespace prefix used in standard LIO-SAM config files. All parameters are optional — unset values retain their defaults. The existing `loadParams(ros::NodeHandle&)` function remains available for ROS-based usage.

### Other Changes

- **C++17 required** — Uses inline variables, structured bindings, and `std::optional` in places
- **PointXYZIRT point type** — Registered via `POINT_CLOUD_REGISTER_POINT_STRUCT` with per-point `ring` (uint16) and `time` (float) fields
- **Global extrinsic variables** — `extRot`, `extRPY`, `extTrans`, `extQRPY` are declared as `inline` globals in `utility.h` for use by `imuConverter()`

## Dependencies

- ROS Noetic
- PCL 1.10+
- OpenCV
- GTSAM 4.x
- Eigen3
- yaml-cpp

## Building

```bash
# As part of a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# clone or symlink this package as lio_sam
catkin_make -DCMAKE_BUILD_TYPE=Release
```

The build produces:
- **Nodelet libraries** — for ROS nodelet manager deployment
- **`liblio_sam_core.so`** — standalone library for non-ROS integration

## Configuration

See `config/params.yaml` for the full parameter reference. Key parameters:

| Parameter | Description |
|-----------|-------------|
| `sensor` | Lidar type: `velodyne`, `ouster`, or `livox` |
| `N_SCAN` | Number of lidar channels (16, 32, 64, 128) |
| `Horizon_SCAN` | Horizontal resolution (Velodyne: 1800, Ouster: 512/1024/2048) |
| `imuAccNoise` | IMU accelerometer white noise |
| `imuGyrNoise` | IMU gyroscope white noise |
| `imuGravity` | Local gravity magnitude (m/s^2) |
| `extrinsicRot` / `extrinsicTrans` | 3x3 rotation and 3x1 translation from lidar to IMU frame |

## Credits

Based on [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) by Tixiao Shan (MIT, IROS 2020).

```
T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus,
"LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping,"
IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020.
```
