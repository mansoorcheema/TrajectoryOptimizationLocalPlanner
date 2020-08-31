cmake_minimum_required(VERSION 3.16)
project(voxblox_test)

set(CMAKE_CXX_STANDARD 14)
set(glog_INCLUDE_DIR /home/mansoor/Downloads/glog-master/build)
#set(glog_DIR_HINT /home/mansoor/Downloads/glog-master/build)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)


include_directories(${CMAKE_SOURCE_DIR}/include)
#include_directories(${Eigen3_INCLUDE_DIR} /usr/local/include/eigen3/ ${glog_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR} /home/mansoor/Downloads/minkindr-master/minkindr/include ${GLOG_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIR})

####################
# SET SOURCE FILES #
####################
set("${PROJECT_NAME}_SRCS"
        src/alignment/icp.cc
        src/core/block.cc
        src/core/esdf_map.cc
        src/core/tsdf_map.cc
        src/integrator/esdf_integrator.cc
        src/integrator/esdf_occ_integrator.cc
        src/integrator/integrator_utils.cc
        src/integrator/intensity_integrator.cc
        src/integrator/tsdf_integrator.cc
        src/io/mesh_ply.cc
        src/io/sdf_ply.cc
        src/mesh/marching_cubes.cc
        src/simulation/objects.cc
        src/simulation/simulation_world.cc
        src/utils/camera_model.cc
        src/utils/evaluation_utils.cc
        src/utils/layer_utils.cc
        src/utils/neighbor_tools.cc
        src/utils/protobuf_utils.cc
        src/utils/timing.cc
        src/utils/voxel_utils.cc
        include/voxblox/Block.pb.cc
        include/voxblox/Layer.pb.cc
        )

add_executable(tsdf_from_pc tsdf_from_pc.cpp ${${PROJECT_NAME}_SRCS})
add_executable(visualize_voxels visualize_voxels.cpp ${${PROJECT_NAME}_SRCS})
add_executable(esdf_from_tsdf esdf_from_tsdf.cpp ${${PROJECT_NAME}_SRCS})
add_executable(mapping EsdfMapping.cpp ${${PROJECT_NAME}_SRCS})

target_link_libraries(tsdf_from_pc Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROTOBUF_LIBRARY})
target_include_directories(tsdf_from_pc PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(visualize_voxels Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROTOBUF_LIBRARY})
target_include_directories(visualize_voxels PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(esdf_from_tsdf Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROTOBUF_LIBRARY})
target_include_directories(esdf_from_tsdf PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})

target_link_libraries(mapping Eigen3::Eigen glog::glog protobuf::libprotobuf ${PROTOBUF_LIBRARY})
target_include_directories(mapping PUBLIC ${EIGEN3_INCLUDE_DIR} ${glog_INCLUDE_DIR})