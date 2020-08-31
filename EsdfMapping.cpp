#include <iostream>
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/esdf_integrator.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/io/layer_io.h"
#include "voxblox/simulation/simulation_world.h"
#include "voxblox/utils/evaluation_utils.h"
#include "voxblox/utils/layer_utils.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace voxblox;  // NOLINT

class Mapping {
public:
    Mapping(std::string obstacles_layer_path, std::string free_space_layer_path) {
        Layer<EsdfVoxel>::Ptr obstacles_layer, free_space_layer;
        //Load layers
        io::LoadLayer<EsdfVoxel>(obstacles_layer_path, &obstacles_layer);
        io::LoadLayer<EsdfVoxel>(free_space_layer_path, &free_space_layer);

        //create esdf maps
        obstacles_map.reset(new EsdfMap(obstacles_layer));
        free_space_map.reset(new EsdfMap(free_space_layer));
    }

    bool getFreespaceDistanceAndGradient( const Eigen::Vector3d& position, double* distance, Eigen::Vector3d* gradient) {
        return free_space_map->getDistanceAndGradientAtPosition(position, distance, gradient);
    }

    bool getFreespaceDistance( const Eigen::Vector3d& position, double* distance) {
        return free_space_map->getDistanceAtPosition(position, distance);
    }

    bool getObstaclesDistanceAndGradient( const Eigen::Vector3d& position, double* distance, Eigen::Vector3d* gradient) {
        return obstacles_map->getDistanceAndGradientAtPosition(position, distance, gradient);
    }

    bool getObstaclesDistance( const Eigen::Vector3d& position, double* distance) {
        return obstacles_map->getDistanceAtPosition(position, distance);
    }

private:
    std::shared_ptr<EsdfMap> obstacles_map;
    std::shared_ptr<EsdfMap> free_space_map;
};

int main() {
    Mapping worldMapping("/home/mansoor/esdf_obstacles_layer.layer","/home/mansoor/esdf_free_layer.layer" );
    Eigen::Vector3d position(1.55,-2.05,1.05);
    double free_space_distance=-1;
    double obstacles_distance = -1;

    worldMapping.getObstaclesDistance(position, &obstacles_distance);
    worldMapping.getFreespaceDistance(position, &free_space_distance);
    return 0;
}
