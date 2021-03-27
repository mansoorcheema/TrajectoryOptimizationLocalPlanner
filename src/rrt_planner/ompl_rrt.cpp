//
// Created by jonasgerstner on 24.02.20.
//
#include "rrt_planner/ompl_rrt.h"

OmplRrt::OmplRrt() :
        lower_bound_(Eigen::Vector3d::Zero()),
        upper_bound_(Eigen::Vector3d::Zero()),
        optimistic_(true),
        num_seconds_to_plan_(2.5),
        reedsshepp_(true),
        turning_radius_(1.0f),
        simplify_solution_(false),
        distance_between_poses_(0.5),
        search_radius_(0.25f),
        state_validity_res_(2.0)  {
}

bool OmplRrt::getPathBetweenWaypoints(const Pose &start, const Pose &goal, std::vector<Pose> &solution, double &reeds_shepp_length, double seconds_to_plan) {
    num_seconds_to_plan_ = seconds_to_plan;
    return getPathBetweenWaypoints(start, goal, solution, reeds_shepp_length);
}

bool OmplRrt::getPathBetweenWaypoints(const Pose &start, const Pose &goal,
                                      std::vector<Pose> &solution, double &reeds_shepp_length) {
    setupFromStartAndGoal(start, goal);

    if (problem_setup_->solve(num_seconds_to_plan_)) {
        if (problem_setup_->haveExactSolutionPath()) {
            if (simplify_solution_) {
                //problem_setup_->reduceVertices();
                problem_setup_->simplifySolution();
            }
            if (verbose_) {
                problem_setup_->getSolutionPath().printAsMatrix(std::cout);
            }
            reeds_shepp_length = problem_setup_->getSolutionPath().length();
        } else {
            //std::cout("OMPL planning failed.");
            return false;
        }
    }
    if (problem_setup_->haveSolutionPath()) {
        solutionPathToTrajectoryPoints(problem_setup_->getSolutionPath(), &solution);
        return true;
    }
    return false;
}

void OmplRrt::setBounds(const Eigen::Vector3d &lower_bound,
                        const Eigen::Vector3d &upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
}

void OmplRrt::setupFromStartAndGoal(
        const Pose &start,
        const Pose &goal) {
    problem_setup_->clear();

    ompl::base::ScopedState<> start_ompl(problem_setup_->getSpaceInformation());
    ompl::base::ScopedState<> goal_ompl(problem_setup_->getSpaceInformation());

    poseToState(start, start_ompl());
    poseToState(goal, goal_ompl());
    problem_setup_->setStartAndGoalStates(start_ompl, goal_ompl);
    problem_setup_->setup();
    if (verbose_) {
        problem_setup_->print();
    }
}

void OmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric &path,
                                             std::vector<Pose> *trajectory_points) const {
    auto num_interpolation_poses = static_cast<unsigned int>(path.length()/distance_between_poses_);
    path.interpolate(num_interpolation_poses);
    trajectory_points->clear();
    trajectory_points->reserve(path.getStateCount());

    std::vector<ompl::base::State *> &state_vector = path.getStates();

    for (ompl::base::State *state_ptr : state_vector) {
        Pose pose;

        stateToPose(state_ptr, pose);

        trajectory_points->emplace_back(pose);
    }
}

const std::shared_ptr<RrtSetup> &OmplRrt::getProblemSetup() const {
    return problem_setup_;
}

void OmplRrt::poseToState(const Pose& pose, ompl::base::State* state){
    auto *s = state->as<ompl::base::SE2StateSpace::StateType>();
    s->setX(pose.x);
    s->setY(pose.y);
    s->setYaw(pose.yaw);
}

void OmplRrt::stateToPose(const ompl::base::State* state, Pose& pose) const{
    auto* s = state->as<ompl::base::SE2StateSpace::StateType>();
    pose.x = s->getX();
    pose.y = s->getY();
    // TODO: find solution for height value
    pose.yaw = s->getYaw();
}
