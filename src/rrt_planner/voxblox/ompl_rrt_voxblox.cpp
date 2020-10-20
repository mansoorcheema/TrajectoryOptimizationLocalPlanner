//
// Created by jonasgerstner on 25.02.20.
//
#include "rrt_planner/voxblox/ompl_rrt_voxblox.h"
#include "rrt_planner/voxblox/rrt_setup_voxblox.h"

VoxbloxOmplRrt::VoxbloxOmplRrt() : OmplRrt()
{
    if (reedsshepp_)
        problem_setup_ = std::make_shared<VoxbloxRrtSetup>(turning_radius_);
    else
        problem_setup_ = std::make_shared<VoxbloxRrtSetup>();
    float dim_x = 4.0f, dim_y = 2.0f, dim_z = 2.0f;

    problem_setup_->setBBDim(dim_x, dim_y, dim_z);
    problem_setup_->setSearchRadius(search_radius_);

    float clearance = 0.1f;
    float wheelbase = 2.875f;
    float track = 1.58f;

    float ratio_trav;
    bool debug_fcl;
    float car_angle_delta;
    bool relax_neighbor_search;

    problem_setup_->setCarConfig(clearance, wheelbase, track);
    problem_setup_->setValConfig(ratio_trav, search_radius_, car_angle_delta, relax_neighbor_search, debug_fcl);
}

void VoxbloxOmplRrt::setTsdfLayer(
        voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
    tsdf_layer_ = tsdf_layer;
    CHECK_NOTNULL(tsdf_layer_);
    voxel_size_ = tsdf_layer_->voxel_size();
}

/**
 * @param current_height set the current height to speed up the nn search in the traversability cloud
 */
void VoxbloxOmplRrt::setupProblem(float current_height = 0.0f) {
    // cast pointer to derived class to keep interfaces clean
    std::dynamic_pointer_cast<VoxbloxRrtSetup>(problem_setup_)->setTsdfVoxbloxCollisionChecking(tsdf_layer_, current_height);

    problem_setup_->setInformedRrtStar();
    problem_setup_->setDefaultObjective();

    if (lower_bound_ != upper_bound_) {
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(0, lower_bound_.x());
        bounds.setLow(1, lower_bound_.y());

        bounds.setHigh(0, upper_bound_.x());
        bounds.setHigh(1, upper_bound_.y());

        // Define start and goal positions.
        problem_setup_->getGeometricComponentStateSpace()
                ->as<ompl::base::ReedsSheppStateSpace>()
                ->setBounds(bounds);
    }

    // This is a fraction of the space extent! Not actual metric units. For
    // mysterious reasons. Thanks OMPL!
    double validity_checking_resolution = 0.01;
    if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
        // If bounds are set, set this to approximately one voxel.
        validity_checking_resolution =
                voxel_size_ / (upper_bound_ - lower_bound_).norm() / state_validity_res_;
    }
    problem_setup_->setStateValidityCheckingResolution(validity_checking_resolution);
}
