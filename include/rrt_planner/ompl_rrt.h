//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_OMPL_RRT_H
#define RRT_PLANNER_OMPL_RRT_H

#include "setup.h"
#include "pose.h"
/**
 * Base class for OMPL
 */
class OmplRrt {
public:
    enum RrtPlannerType {
        kRrt = 0,
        kRrtConnect,
        kRrtStar,
        kInformedRrtStar,
        kBitStar,
        kPrm
    };

    OmplRrt();
    virtual ~OmplRrt() {}

    virtual void setupProblem(float current_height) = 0;

    void setBounds(const Eigen::Vector3d& lower_bound,
                   const Eigen::Vector3d& upper_bound);

    bool getPathBetweenWaypoints(const Pose& start, const Pose& goal, std::vector<Pose>& solution, double& reeds_shepp_length);
    bool getPathBetweenWaypoints(const Pose& start, const Pose& goal, std::vector<Pose>& solution, double& reeds_shepp_length, double seconds_to_plan);

    void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path, std::vector<Pose>* trajectory_points) const;

    inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }

    const std::shared_ptr<RrtSetup> &getProblemSetup() const;

    static void poseToState(const Pose& pose, ompl::base::State*);
    void stateToPose(const ompl::base::State*, Pose&) const;

    virtual bool mapsReady() const = 0;

protected:
    void setupFromStartAndGoal(const Pose& start, const Pose& goal);

    Eigen::Vector3d lower_bound_;
    Eigen::Vector3d upper_bound_;

    std::shared_ptr<RrtSetup> problem_setup_;

    RrtPlannerType planner_type_;
    double num_seconds_to_plan_;
    bool simplify_solution_;
    bool verbose_;
    bool optimistic_;
    bool reedsshepp_;

    float turning_radius_;
    float search_radius_;

    double distance_between_poses_;
    double state_validity_res_;
};
#endif //RRT_PLANNER_OMPL_RRT_H
