#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace voxblox;  // NOLINT


bool visualizeTsdfVoxels(const TsdfVoxel& voxel, const Point& /*coord*/,
                                Color* color) {
    CHECK_NOTNULL(color);
    constexpr float kMinWeight = 1e-2;
    if (voxel.weight > kMinWeight) {
        *color = voxel.color;
        return true;
    }
    return false;
}

bool visualizeDistanceIntensityTsdfVoxels(const TsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
    CHECK_NOTNULL(intensity);
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

bool visualizeDistanceIntensityEsdfVoxels(const EsdfVoxel& voxel,
                                          const Point& /*coord*/,
                                          double* intensity) {
    CHECK_NOTNULL(intensity);
    if (voxel.observed) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

template <typename VoxelType>
void createColorPointcloudFromLayer(
        const Layer<VoxelType>& layer,
        Pointcloud* ptcloud, Colors* colors) {
    CHECK_NOTNULL(ptcloud);
    CHECK_NOTNULL(colors);
    ptcloud->clear();
    colors->clear();
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Temp variables.
    Color color;
    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (visualizeTsdfVoxels(block.getVoxelByLinearIndex(linear_index), coord,
                             &color)) {
                ptcloud->push_back(coord);
                colors->push_back(color);
            }
        }
    }
}

int main() {
    FloatingPoint voxel_size_(0.10);
    int voxels_per_side_(16);
    FloatingPoint truncation_distance_ = 4 * voxel_size_;

    TsdfIntegratorBase::Config config;
    config.default_truncation_distance = truncation_distance_;
    config.integrator_threads = 1;

    //setup world
    SimulationWorld world_;
    Point min_bound(-5.0, -5.0, -1.0);
    Point max_bound(5.0, 5.0, 6.0);
    world_.setBounds(min_bound, max_bound);

    // add cylinder as obstacle
    Point cylinder_center(0.0, 2.0, 0.0);
    FloatingPoint cylinder_radius = 2;
    FloatingPoint cylinder_height = 2;
    world_.addObject(std::unique_ptr<Object>(new Cylinder(
            cylinder_center, cylinder_radius, cylinder_height, Color::Red())));

    // add road
    Point cube_center(0.0, 0.0, 0.0);
    Point cube_size(25.0, 8.0, 0.2);
    Color road_color(121, 104, 120);
    world_.addObject(std::unique_ptr<Object>(new Cube(
            cube_center,cube_size, road_color)));

    //add ground
    world_.addGroundLevel(0.0);

    //setup poses
    AlignedVector<Transformation> poses_;
    // Next, generate poses evenly spaced in a circle around the object.
    FloatingPoint radius = 6.0;
    FloatingPoint height = 2.0;
    int num_poses = 20;  // static_cast<int>(200 * voxel_size_);
    poses_.reserve(num_poses);

    FloatingPoint max_angle = 2 * M_PI;
    FloatingPoint angle_increment = max_angle / num_poses;

    bool render_cameras = false;
    bool render_road = true;
    bool render_obstacles = true;

    for (FloatingPoint angle = 0.0; angle < max_angle;
         angle += angle_increment) {
        // Generate a transformation to look at the center pose.
        Point position(radius * sin(angle), radius * cos(angle), height);
        Point facing_direction = cylinder_center - position;

        FloatingPoint desired_yaw = 0.0;
        if (std::abs(facing_direction.x()) > 1e-4 ||
            std::abs(facing_direction.y()) > 1e-4) {
            desired_yaw = atan2(facing_direction.y(), facing_direction.x());
        }

//        if(render_cameras) {
//            myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
//            myfile << (int) 0 << ";" << (int) 0 << ";" << (int) 0 << endl;
//        }
        // Face the desired yaw and pitch forward a bit to get some of the floor.
        Quaternion rotation =
                Quaternion(Eigen::AngleAxis<FloatingPoint>(-0.1, Point::UnitY())) *
                Eigen::AngleAxis<FloatingPoint>(desired_yaw, Point::UnitZ());

        poses_.emplace_back(Transformation(rotation, position));
    }


    // Simple integrator
    Layer<TsdfVoxel> simple_layer(voxel_size_, voxels_per_side_);
    SimpleTsdfIntegrator simple_integrator(config, &simple_layer);

    Eigen::Vector2i depth_camera_resolution_(Eigen::Vector2i(320, 240));
    FloatingPoint fov_h_rad_(2.61799);
    FloatingPoint max_dist_(10.0);

//    Point p(10,0,0);
//
//    Colors  clrs;
//    clrs.push_back(Color());
//
//    auto t = Transformation(Quaternion::Identity(),Point(0,0,0));
//
//    Pointcloud ptcloud;
//    ptcloud.push_back(p);
//
//    simple_integrator.integratePointCloud(t, ptcloud,clrs );

    // run ntegrator
    for (size_t i = 0; i < poses_.size(); i++) {
        Pointcloud ptcloud, ptcloud_C;
        Colors colors;

        world_.getPointcloudFromTransform(poses_[i], depth_camera_resolution_,
                                          fov_h_rad_, max_dist_, &ptcloud, &colors);
//        for (size_t i = 0; i < ptcloud.size(); ++i) {
//            auto position = ptcloud[i];
//            auto color = colors[i];
//            if(render_road  && (color==road_color)) {
//                myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
//                myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;
//            }
//
//            if(render_obstacles  && !(color==road_color)) {
//                myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
//                myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;
//            }
//        }

        transformPointcloud(poses_[i].inverse(), ptcloud, &ptcloud_C);


        simple_integrator.integratePointCloud(poses_[i], ptcloud_C, colors);
    }

//    Pointcloud ptcloud;
//    Colors colors;
//    createColorPointcloudFromLayer(simple_layer, &ptcloud, &colors );
//
//    ofstream myfile;
//    myfile.open ("/home/mansoor/tsdf_pointcloud.txt");
//
//    for (size_t i = 0; i < ptcloud.size(); ++i) {
//        auto position = ptcloud[i];
//        auto color = colors[i];
//        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
//        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;
//
//        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
//        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;
//    }
//    myfile.close();
    voxblox::io::SaveLayer(simple_layer, "/home/mansoor/tsdf_layer");
    std::cout << "Done!" << std::endl;
    return 0;
}
