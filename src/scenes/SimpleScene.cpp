//
// Created by mansoor on 13.11.20.
//

#include "scenes/SimpleScene.h"
    void scenes::SimpleScene::_setupObjects() {
        // set ground level
        world_.addGroundLevel(0.0);

        // add a cylinder obstacle on road
        world_.addObject(std::unique_ptr<Object>(new Cylinder(
                obstacle_center_, obstacle_radius_, obstacle_height_, Color::Red())));

        // add road surface
        world_.addObject(std::unique_ptr<Object>(new Cube(
                road_center_, road_size_, road_color_)));
    }
