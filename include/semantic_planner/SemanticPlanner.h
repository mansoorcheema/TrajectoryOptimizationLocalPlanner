//
// Created by mansoor on 13.11.20.
//

#ifndef TRAJECTORY_PLANNER_SEMANTICPLANNER_H
#define TRAJECTORY_PLANNER_SEMANTICPLANNER_H

#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"
#include "loco_planner/loco.h"
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

using namespace std;
using namespace voxblox;

#define kN 10 //polynomial degree
#define kD 3 //state space
#define kS 3 //number of segments

namespace semantic_planner {


    class SemanticPlanner {
    public:
        SemanticPlanner(const EsdfMap& obstacles_map, const EsdfMap& drivable_map);

        SemanticPlanner(const std::string obstacles_layer_path, const std::string drivable_layer_path);

        void setupPlanner();

        mav_trajectory_generation::Trajectory
        planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal, std::vector<Eigen::VectorXd> *waypoints,
                       FloatingPoint dt = 0.1, bool verbose = true);
        void savePlannedPath(const std::vector<Eigen::VectorXd>& waypoints,std::string point_cloud_path );


    private:
        double _getMapDistanceAndGradient(
                const Eigen::Vector3d &position, Eigen::Vector3d *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
                double d);

        double _getMapDistanceAndGradientVector(
                const Eigen::VectorXd &position, Eigen::VectorXd *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
                double d);

    private:
        // ESDF layers
        Layer<EsdfVoxel>::Ptr drivable_layer_;
        Layer<EsdfVoxel>::Ptr obstacles_layer_;

        // ESDF maps
        voxblox::EsdfMap::Ptr esdf_obstacles_map_;
        voxblox::EsdfMap::Ptr esdf_drivable_map_;

        // Planner
        loco_planner::Loco<kN> loco_;
        // conservative planner for obstacles.  unknown space means obstacle is 0 - meter awau
        static constexpr int DEFAULT_DISTANCE_OBSTACLES = 0.0;
        // conservative. Unkonwn points are far away from road.
        static constexpr int DEFAULT_DISTANCE_DRIVINGZONE = 3.0;
    };
}
#endif //TRAJECTORY_PLANNER_SEMANTICPLANNER_H
