cmake_minimum_required(VERSION 2.8.3)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  nodelet
  pluginlib
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  nodelet
  roscpp
  pluginlib
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

##### cuda_plane_line_odometry #####
# find_package(CUDA REQUIRED)
# message(STATUS "CUDA_LIBRARIES : ${CUDA_LIBRARIES}")
# message(STATUS "CUDA_cublas_LIBRARY : ${CUDA_cublas_LIBRARY}")
# message(STATUS "CUDA_cusolver_LIBRARY : ${CUDA_cusolver_LIBRARY}")
# message(STATUS "CUDA_INCLUDE_DIRS : ${CUDA_INCLUDE_DIRS}")
# include_directories(${CUDA_INCLUDE_DIRS})
# add_subdirectory(src/cuda_plane_line_odometry)
##### cuda_plane_line_odometry #####

###########
## Build ##
###########

# Range Image Projection
# add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
# add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
# add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
# add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
# add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
# add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
# target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)
##### cuda_plane_line_odometry #####
# target_link_libraries(${PROJECT_NAME}_mapOptmization CUDALinePlaneOdometry)
##### cuda_plane_line_odometry #####

# IMU Preintegration
# add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
# target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

### nodelet test
# add_library(${PROJECT_NAME} 
#   src/featureExtraction.cpp
#   src/imageProjection.cpp
#   src/imuPreintegration.cpp
#   src/mapOptmization.cpp
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} ${OpenCV_LIBRARIES} gtsam)
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
# target_compile_options(${PROJECT_NAME}  PRIVATE ${OpenMP_CXX_FLAGS})


add_library (${PROJECT_NAME}_FE src/featureExtraction.cpp)
target_link_libraries(${PROJECT_NAME}_FE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library (feature_extraction_nodelet src/nodelets/featureExtractionNodelet.cpp)
add_dependencies(feature_extraction_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(feature_extraction_nodelet ${PROJECT_NAME}_FE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library (${PROJECT_NAME}_IP src/imageProjection.cpp)
target_link_libraries(${PROJECT_NAME}_IP ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library (image_projection_nodelet src/nodelets/imageProjectionNodelet.cpp)
add_dependencies(image_projection_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(image_projection_nodelet ${PROJECT_NAME}_IP  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library (${PROJECT_NAME}_IMU_P src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_IMU_P ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_library (imu_preintegration_nodelet src/nodelets/imuPreintegrationNodelet.cpp)
add_dependencies(imu_preintegration_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(imu_preintegration_nodelet ${PROJECT_NAME}_IMU_P ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)


add_library (${PROJECT_NAME}_MO src/mapOptmization.cpp)
target_compile_options(${PROJECT_NAME}_MO PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_MO ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_library (map_optimization_nodelet src/nodelets/mapOptimizationNodelet.cpp)
add_dependencies(map_optimization_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(map_optimization_nodelet PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(map_optimization_nodelet ${PROJECT_NAME}_MO ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)
# #### 


install(TARGETS ${PROJECT_NAME}_FE  ${PROJECT_NAME}_IP ${PROJECT_NAME}_IMU_P ${PROJECT_NAME}_MO feature_extraction_nodelet image_projection_nodelet imu_preintegration_nodelet map_optimization_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)