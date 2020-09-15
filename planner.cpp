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
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include "loco_planner/loco.h"

using namespace std;
namespace fs = std::experimental::filesystem;
using namespace voxblox;  // NOLINT

static constexpr int kN = 10;
static constexpr int kD = 3;

std::shared_ptr<voxblox::EsdfMap> esdf_map_;
std::shared_ptr<voxblox::EsdfMap> esdf_free_map_;

double getMapDistanceAndGradient(
        const Eigen::Vector3d& position, Eigen::Vector3d* gradient) {
    double distance = 3.0;
    const bool kInterpolate = false;
    if (!esdf_map_->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                     &distance, gradient)) {
        return 3.0;
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
    io::LoadLayer<EsdfVoxel>("/home/mansoor/esdf_free_layer.layer", &layer_from_file);

    // Planner.
    loco_planner::Loco<kN> loco_(kD) ;
    // Map.
    esdf_map_.reset(new EsdfMap(layer_from_file));

    Eigen::Vector3d  v(-7., -1.25, 2);
    Eigen::Vector3d w(7.25, -1.5, 1);
    loco_.setupFromPositions(v, w, 3, 10.0);

    loco_.setDistanceAndGradientFunction(&getMapDistanceAndGradientVector);
    loco_.setMapResolution(esdf_map_->voxel_size());

    //double first_solve_cost = loco_.computeCollisionCostAndGradient(nullptr);

    loco_.solveProblem();
    //double first_solve_cost = loco_.computeTotalCostAndGradients(nullptr);

    // Save success and Trajectory.
    mav_trajectory_generation::Trajectory trajectory;
    loco_.getTrajectory(&trajectory);

    // sample trajectory
    std::vector<Eigen::VectorXd> position;

    trajectory.evaluateRange(trajectory.getMinTime(), trajectory.getMaxTime(), 0.1,
                             mav_trajectory_generation::derivative_order::POSITION, &position);

    std::string point_cloud_path = "/home/mansoor/pointcloud.txt";
    std::string point_cloud_trajectory = point_cloud_path.substr(0,point_cloud_path.length()-4) + "_plan.txt";
    fs::copy(point_cloud_path, point_cloud_trajectory,fs::copy_options::overwrite_existing);
    std::ofstream myfile;
    myfile.open (point_cloud_trajectory, std::ofstream::out | std::ofstream::app);

    Eigen::VectorXd grad_a, gradient;

    for(auto p: position) {
        myfile << p(0) << ";" << p(1) << ";" << p(2) << ";";
        myfile << 0 << ";" << 0 << ";" << 255 << endl;


        double cost_p = loco_.computeFreeCostAndGradient(p.head<3>(), &gradient);
        std::cout<< cost_p<<std::endl<<std::endl;
    }
    myfile.close();
    std::cout << "Done!" << std::endl;
    return 0;
}
