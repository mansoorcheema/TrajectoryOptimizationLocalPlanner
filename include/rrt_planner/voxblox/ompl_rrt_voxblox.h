//
// Created by jonasgerstner on 24.02.20.
//
#ifndef RRT_PLANNER_OMPL_RRT_VOXBLOX_H
#define RRT_PLANNER_OMPL_RRT_VOXBLOX_H

//#include <voxblox_ros/esdf_server.h>
#include <rrt_planner/ompl_rrt.h>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

class VoxbloxOmplRrt : public OmplRrt{
public:
    VoxbloxOmplRrt();
    void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);
    void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);

    void setupProblem(float current_height) override;

    bool mapsReady() const override{return true;}

protected:
    double voxel_size_;

    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
    voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
};
#endif //RRT_PLANNER_OMPL_RRT_VOXBLOX_H
