#include <iostream>
#include "voxblox/core/voxel.h"
#include "scenes/SimpleScene.h"
#include "scenes/BaseScene.h"

int main(int argc, char *argv[]) {
    if (argc < 4) {
        std::cout<<"Usage: generate_tsdf <scene> <output_obstacles_layer> <output_drivable_layer>";
        std::cout<<"scene - Scenario to use for apth planning. From Simple and Complex scenarios"<<std::endl;
        std::cout<<"output_drivable_layer - Path to load the TSDF layer for driving zone"<<std::endl;
        std::cout<<"output_obstacles_layer - Path to save the TSDF layer for obstacles"<<std::endl;
        return -1;
    }

    scenes::BaseScene*  scene;

    if (strcmp(argv[1], "simple") == 0) {
        scene = new scenes::SimpleScene();
    }

    scene->setupScene();
    scene->saveSceneVoxelPointcloud(argv[1], argv[2]);

    delete scene;
    return 0;
}
