#!/bin/bash
# remove previous output
rm -rf output
mkdir output

# switch to build directory
if [ -d build ]
then
    echo "moving to build directory"
    cd build
    make
else
    echo "Compiling code..."
    bash scripts/compile.sh
    cd build
fi

#generate tsdf and pointcloud
echo "generating tsdf's and scene pointcloud"
./generate_tsdf simple ../output/tsdf_obstacles_layer.layer ../output/tsdf_free_layer.layer ../output/pointcloud_simple.txt

# generate esdf
echo "generating esdf's as pointcloud"
./esdf_from_tsdf ../output/tsdf_free_layer.layer ../output/esdf_free_layer.layer free
./esdf_from_tsdf ../output/tsdf_obstacles_layer.layer ../output/esdf_obstacles_layer.layer obstacles

# save esdf voxels as point cloud
echo "saving esdf's as pointcloud"
./visualize_esdf_voxels ../output/esdf_obstacles_layer.layer obstacles
./visualize_esdf_voxels ../output/esdf_free_layer.layer free

# planning
echo "planning path"
./planning -7,-0.2,0.5 7,-0.7,0.5 ../output/esdf_obstacles_layer.layer ../output/esdf_free_layer.layer ../output/pointcloud_simple.txt