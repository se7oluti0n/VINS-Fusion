cmake_minimum_required(VERSION 3.5)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     nav_msgs
#     camera_models
#     cv_bridge
#     roslib
#     )
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_models REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV)


find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
# set(camera_models_INCLUDE_DIRS "${camera_models_DIR}/../../../include")
# set(camera_models_LIBRARY_DIRS "${camera_models_DIR}/../../../lib")

message("include: ${camera_models_INCLUDE_DIR}")
include_directories(${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

# catkin_package()

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

set(dependencies 
  rclcpp
  std_msgs
  nav_msgs
  camera_models
  sensor_msgs
  cv_bridge
  geometry_msgs
  visualization_msgs
)


target_link_libraries(loop_fusion_node ${OpenCV_LIBS} ${CERES_LIBRARIES} ) 
ament_target_dependencies(loop_fusion_node
  ${dependencies}
)

install(TARGETS
  loop_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY support_files
  DESTINATION share/${PROJECT_NAME}/
)


# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(${dependencies})
ament_package()