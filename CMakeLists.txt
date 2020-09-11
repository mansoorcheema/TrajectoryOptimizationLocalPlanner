cmake_minimum_required(VERSION 3.16)
project(voxblox_test)

set(CMAKE_CXX_STANDARD 17)
#set(glog_INCLUDE_DIR /home/mansoor/Downloads/glog-master/build)
#set(ceres_DIR /home/mansoor/Downloads/ceres-solver-1.14.0/build)
#set(glog_DIR_HINT /home/mansoor/Downloads/glog-master/build)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CERES_INCLUDE_DIRS})
#include_directories(${Eigen3_INCLUDE_DIR} /usr/local/include/eigen3/ ${glog_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR} /home/mansoor/Opensource/minkindr/minkindr/include ${GLOG_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


####################
# SET SOURCE FILES #
####################
set("${PROJECT_NAME}_SRCS"
        src/voxblox/alignment/icp.cc
        src/voxblox/core/block.cc
        src/voxblox/core/esdf_map.cc
        src/voxblox/core/tsdf_map.cc
        src/voxblox/integrator/esdf_integrator.cc
        src/voxblox/integrator/esdf_occ_integrator.cc
        src/voxblox/integrator/integrator_utils.cc
        src/voxblox/integrator/intensity_integrator.cc
        src/voxblox/integrator/tsdf_integrator.cc
        src/voxblox/io/mesh_ply.cc
        src/voxblox/io/sdf_ply.cc
        src/voxblox/mesh/marching_cubes.cc
        src/voxblox/simulation/objects.cc
        src/voxblox/simulation/simulation_world.cc
        src/voxblox/utils/camera_model.cc
        src/voxblox/utils/evaluation_utils.cc
        src/voxblox/utils/layer_utils.cc
        src/voxblox/utils/neighbor_tools.cc
        src/voxblox/utils/protobuf_utils.cc
        src/voxblox/utils/timing.cc
        src/voxblox/utils/voxel_utils.cc
        src/loco_planner/loco_main.cpp
        src/mav_trajectory_generation/motion_defines.cpp
        src/mav_trajectory_generation/polynomial.cpp
        src/mav_trajectory_generation/polynomial_timing_evaluation.cpp
        src/mav_trajectory_generation/rpoly/rpoly_ak1.cpp
        src/mav_trajectory_generation/segment.cpp
        src/mav_trajectory_generation/timing.cpp
        src/mav_trajectory_generation/trajectory.cpp
        src/mav_trajectory_generation/vertex.cpp
        )

set(PROTO_DEFNS proto/voxblox/Block.proto proto/voxblox/Layer.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})

add_executable(tsdf_from_pc tsdf_from_pc.cpp ${${PROJECT_NAME}_SRCS})
add_executable(visualize_voxels visualize_voxels.cpp ${${PROJECT_NAME}_SRCS})
add_executable(esdf_from_tsdf esdf_from_tsdf.cpp ${${PROJECT_NAME}_SRCS})
add_executable(mapping EsdfMapping.cpp ${${PROJECT_NAME}_SRCS})
add_executable(planning planner.cpp ${${PROJECT_NAME}_SRCS})

target_link_libraries(tsdf_from_pc Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto)
#target_include_directories(tsdf_from_pc PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(visualize_voxels Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto ${CERES_LIBRARIES})
target_link_libraries(planning Eigen3::Eigen glog::glog protobuf::libprotobuf  ${PROJECT_NAME}_proto ${CERES_LIBRARIES} "stdc++fs")
#target_include_directories(visualize_voxels PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(esdf_from_tsdf Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROJECT_NAME}_proto)
#target_include_directories(esdf_from_tsdf PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(mapping Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROTOBUF_LIBRARIES})
#target_include_directories(mapping PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})