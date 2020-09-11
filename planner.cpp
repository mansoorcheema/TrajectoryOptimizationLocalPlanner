#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include <voxblox/mesh/mesh_integrator.h>
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"
#include <voxblox/io/mesh_ply.h>
#include <loco_planner/loco.h>
#include <iostream>
#include <fstream>
#include "loco_planner/loco.h"

using namespace std;
using namespace voxblox;  // NOLINT

static constexpr int kN = 10;
static constexpr int kD = 3;

std::shared_ptr<voxblox::EsdfMap> esdf_map_;

double getMapDistanceAndGradient(
        const Eigen::Vector3d& position, Eigen::Vector3d* gradient) {
    double distance = 0.0;
    const bool kInterpolate = false;
    if (!esdf_map_->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                     &distance, gradient)) {
        return 0.0;
    }
    return distance;
}

double getMapDistanceAndGradientVector(
        const Eigen::VectorXd& position, Eigen::VectorXd* gradient) {
    CHECK_EQ(position.size(), 3);
    if (gradient == nullptr) {
        return getMapDistanceAndGradient(position, nullptr);
    }
    Eigen::Vector3d gradient_3d;
    double distance = getMapDistanceAndGradient(position, &gradient_3d);
    *gradient = gradient_3d;
    return distance;
}



int main() {
    Layer<EsdfVoxel>::Ptr layer_from_file;
    io::LoadLayer<EsdfVoxel>("../data/esdf_obstacles_layer.layer", &layer_from_file);

    // Planner.
    loco_planner::Loco<kN> loco_(kD) ;
    // Map.
    esdf_map_.reset(new EsdfMap(layer_from_file));

    Eigen::Vector3d  v(-5.0,2.0,2);
    Eigen::Vector3d w(5.0,2.0,2);
    loco_.setupFromPositions(v, w, 3, 10.0);

    loco_.setDistanceAndGradientFunction(&getMapDistanceAndGradientVector);
    loco_.setMapResolution(esdf_map_->voxel_size());

    //double first_solve_cost = loco_.computeCollisionCostAndGradient(nullptr);

    Eigen::VectorXd x0, x;
    loco_.getParameterVector(&x0);
    x = x0;
    loco_.solveProblem();
    //double first_solve_cost = loco_.computeTotalCostAndGradients(nullptr);

    // Save success and Trajectory.
    mav_trajectory_generation::Trajectory trajectory;
    loco_.getTrajectory(&trajectory);

    // sample trajectory
    std::vector<Eigen::VectorXd> position;

    trajectory.evaluateRange(trajectory.getMinTime(), trajectory.getMaxTime(), 1,
                             mav_trajectory_generation::derivative_order::POSITION, &position);

    for(auto p: position) {
        std::cout<< p.head<3>()<<std::endl<<std::endl;
    }
    std::cout << "Done!" << std::endl;
    return 0;
}
