//
// Created by mansoor on 13.11.20.
//
#include "semantic_planner/SemanticPlanner.h"

semantic_planner::SemanticPlanner::SemanticPlanner(const EsdfMap& obstacles_map, const EsdfMap& drivable_map) : loco_(kD) {
    // Load ESDF Map
    esdf_obstacles_map_.reset(new EsdfMap(obstacles_map));
    esdf_drivable_map_.reset(new EsdfMap(drivable_map));

    assert(esdf_obstacles_map_->voxel_size() == esdf_drivable_map_->voxel_size());
    setupPlanner();
}

semantic_planner::SemanticPlanner::SemanticPlanner(const std::string obstacles_layer_path, const std::string drivable_layer_path) : loco_(
        kD) {
    // Load layers
    io::LoadLayer<EsdfVoxel>(obstacles_layer_path, &obstacles_layer_);
    io::LoadLayer<EsdfVoxel>(drivable_layer_path, &drivable_layer_);

    // Load ESDF Map
    esdf_obstacles_map_.reset(new EsdfMap(obstacles_layer_));
    esdf_drivable_map_.reset(new EsdfMap(drivable_layer_));

    assert(esdf_obstacles_map_->voxel_size() == esdf_drivable_map_->voxel_size());
    setupPlanner();
}

void semantic_planner::SemanticPlanner::setupPlanner() {
    // Setup distance functions for obstacles and driving space
    loco_.setDistanceAndGradientFunction([this](const Eigen::VectorXd &position, Eigen::VectorXd *gradient) {
        return _getMapDistanceAndGradientVector(position, gradient, esdf_obstacles_map_, DEFAULT_DISTANCE_OBSTACLES);
    });

    loco_.setFreeDistanceAndGradientFunction([this](const Eigen::VectorXd &position, Eigen::VectorXd *gradient) {
        return _getMapDistanceAndGradientVector(position, gradient, esdf_drivable_map_,
                                                DEFAULT_DISTANCE_DRIVINGZONE);
    });

    loco_.setMapResolution(esdf_obstacles_map_->voxel_size());
}

mav_trajectory_generation::Trajectory
semantic_planner::SemanticPlanner::planTrajectory(const Eigen::Vector3d start, const Eigen::Vector3d goal,
                                std::vector<Eigen::VectorXd> *waypoints,
                                FloatingPoint dt, bool verbose) {
    loco_.setupFromPositions(start, goal, kS, 10.0);
    loco_.solveProblem();
    // Save success and Trajectory.
    mav_trajectory_generation::Trajectory trajectory;
    loco_.getTrajectory(&trajectory);

    trajectory.evaluateRange(trajectory.getMinTime(), trajectory.getMaxTime(), dt,
                             mav_trajectory_generation::derivative_order::POSITION, waypoints);
    if (verbose) {
        double total_collision_cost = loco_.computeCollisionCostAndGradient(nullptr);
        double total_collision_free = loco_.computeFreeCostAndGradient(nullptr);
        double total_derivative_costs = loco_.computeDerivativeCostAndGradient(nullptr);
        std::cout << "Costs Report:" << std::endl;
        std::cout << "total_collision_cost:" << total_collision_cost << std::endl;
        std::cout << "total_collision_free:" << total_collision_free << std::endl;
        std::cout << "total_derivative_costs:" << total_derivative_costs << std::endl;
    }
    return std::move(trajectory);
}

void semantic_planner::SemanticPlanner::savePlannedPath(const std::vector<Eigen::VectorXd>& waypoints,std::string point_cloud_path ) {
    std::string point_cloud_trajectory = point_cloud_path.substr(0, point_cloud_path.length() - 4) + "_plan.txt";
    fs::copy(point_cloud_path, point_cloud_trajectory, fs::copy_options::overwrite_existing);
    std::ofstream pointcloud_file;
    pointcloud_file.open(point_cloud_trajectory, std::ofstream::out | std::ofstream::app);
    for (auto p: waypoints) {
        pointcloud_file << p(0) << ";" << p(1) << ";" << p(2) + 0.15f << ";";
        pointcloud_file << 0 << ";" << 0 << ";" << 255 << endl;
    }
    pointcloud_file.close();
}

double semantic_planner::SemanticPlanner::_getMapDistanceAndGradient(
        const Eigen::Vector3d &position, Eigen::Vector3d *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
        double d) {
    double distance = d;
    const bool kInterpolate = false;
    if (!esdf_map->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                    &distance, gradient)) {
        return d;
    }
    return distance;
}

double semantic_planner::SemanticPlanner::_getMapDistanceAndGradientVector(
        const Eigen::VectorXd &position, Eigen::VectorXd *gradient, std::shared_ptr<voxblox::EsdfMap> esdf_map,
        double d) {
    CHECK_EQ(position.size(), 3);
    if (gradient == nullptr) {
        return _getMapDistanceAndGradient(position, nullptr, esdf_map, d);
    }
    Eigen::Vector3d gradient_3d;
    double distance = _getMapDistanceAndGradient(position, &gradient_3d, esdf_map, d);
    *gradient = gradient_3d;
    return distance;
}
