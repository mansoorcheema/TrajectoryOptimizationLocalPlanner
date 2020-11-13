//
// Created by mansoor on 13.11.20.
//

#ifndef VOXBLOX_TEST_BASESCENE_H
#define VOXBLOX_TEST_BASESCENE_H

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
using namespace voxblox;

#define DEBUG_SCENE 1

namespace scenes{
    class BaseScene {
    public:

        BaseScene(const Point min_bound, const Point max_bound,
                  const size_t voxels_per_side, const FloatingPoint voxel_size,
                  const FloatingPoint fov_h_rad, const FloatingPoint max_dist,
                  const FloatingPoint sensor_noise, const Eigen::Vector2i depth_camera_resolution) :
                min_bound_(min_bound),
                max_bound_(max_bound),
                voxels_per_side_(voxels_per_side),
                voxel_size_(voxel_size),
                fov_h_rad_(fov_h_rad),
                max_dist_(max_dist),
                sensor_noise_(sensor_noise),
                depth_camera_resolution_(depth_camera_resolution),
                generated_(false) {

            reset();
        }

        BaseScene() : min_bound_(-5.0, -5.0, -1.0),
                      max_bound_(5.0, 5.0, 6.0),
                      voxels_per_side_(16),
                      voxel_size_(0.10),
                      fov_h_rad_(2.61799),
                      max_dist_(10.0),
                      sensor_noise_(0.0),
                      generated_(false),
                      depth_camera_resolution_(Eigen::Vector2i(320, 240)) {
            reset();
        }

        void reset();

        const std::shared_ptr<Layer<TsdfVoxel>> &getObstaclesLayer() const {
            return obstacles_layer_;
        }

        const std::shared_ptr<Layer<TsdfVoxel>> &getDrivableZoneLayer() const {
            return drivable_zone_layer_;
        }

        FloatingPoint getVoxelSize() const {
            return voxel_size_;
        }

        void setupScene();

        void saveSceneVoxelPointcloud(const std::string obstacles_layer,
                                                         const std::string drivable_zone_layer) const;

    private:
        void getPoses(AlignedVector<Transformation> *poses,
                      const Point &start = Point(-13., -1, 3),
                      const Point &inc = Point(1, 0, 0),
                      std::size_t num_poses = 20);

    protected:
        // whether this point belong to drivable surface semantic category
        virtual bool isDrivable(const Point &p, const Color &c) = 0;

    protected:
        // world
        SimulationWorld world_;
        Point min_bound_;
        Point max_bound_;
        FloatingPoint voxel_size_;
        size_t voxels_per_side_;
        //AlignedVector<Transformation> poses_;
        Color drivableZone_;

        //config
        TsdfIntegratorBase::Config tsdf_layer_config_;

        // camera config
        Eigen::Vector2i depth_camera_resolution_;
        FloatingPoint fov_h_rad_;
        FloatingPoint max_dist_;
        FloatingPoint sensor_noise_;

        // tsdf layers
        std::shared_ptr<Layer<TsdfVoxel>> obstacles_layer_;
        std::shared_ptr<Layer<TsdfVoxel>> drivable_zone_layer_;

        // tsdf integrators
        std::shared_ptr<SimpleTsdfIntegrator> obstacles_integrator_;
        std::shared_ptr<SimpleTsdfIntegrator> drivable_zone_integrator_;

        //other
        bool generated_;// whether esdf layers are initialized and updated
    };
}


#endif //VOXBLOX_TEST_BASESCENE_H
