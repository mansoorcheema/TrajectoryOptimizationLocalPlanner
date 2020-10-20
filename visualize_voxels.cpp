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
#include <iostream>
#include <fstream>

using namespace std;
using namespace voxblox;  // NOLINT

bool visualizeVoxels(const TsdfVoxel& voxel,
                                Color* color) {
    CHECK_NOTNULL(color);
//    if((int)voxel.color.r==255 && (int)voxel.color.g== 255 && (int)voxel.color.b == 255) {
//    if(!(voxel.color == Color::Red() || voxel.color==Color(121, 104, 120) || voxel.color == Color(108,180,39))) {
//        cout << "Distance:" << voxel.distance << endl;
//        cout << "Weight:" << voxel.weight << endl;
//        //return false;
//    }

//    if (voxel.distance >=0.4f)
//        return false;
//


    constexpr float kMinWeight = 0.1;
    if (voxel.weight > kMinWeight) {
        *color = voxel.color;
        return true;
    }
    return false;
}

bool visualizeDistanceIntensityTsdfVoxels(const TsdfVoxel& voxel,
                                                 const Point& /*coord*/,
                                                 double* intensity) {
    CHECK_NOTNULL(intensity);
    constexpr float kMinWeight = 1e-3;
    if (voxel.weight > kMinWeight) {
        *intensity = voxel.distance;
        return true;
    }
    return false;
}

bool visualizeVoxels(const EsdfVoxel& voxel, Color* color) {
    //CHECK_NOTNULL(intensity);
    if (voxel.observed) {
        //*intensity = voxel.distance;
        //std::cout<<voxel.distance<<std::endl;

        if(voxel.distance  > 1  && voxel.distance < 2 ) {
            //std::cout << voxel.distance << std::endl;
            *color = Color::Yellow();
            //color->a = 200-uint8_t (voxel.distance*100);
            return true;
        } else if(voxel.distance > 0.5 && voxel.distance <= 1) {
            //std::cout << voxel.distance << std::endl;
            *color = Color::Orange();
            return true;
        } else if(voxel.distance > 0 && voxel.distance <= 0.5) {
            //std::cout << voxel.distance << std::endl;
            *color = Color::Red();
            return true;
        } else if(voxel.distance >= 2) {
            //std::cout << voxel.distance << std::endl;
            *color = Color::Green();
            return true;
        } else if(voxel.distance <= 0) {
            //std::cout << voxel.distance << std::endl;
            *color = Color::Gray();
            return true;
        }
    }
    return false;
}

template <typename VoxelType>
void createColorPointcloudFromLayer(
        const Layer<VoxelType>& layer,
        Pointcloud* ptcloud, Colors* colors) {
    CHECK_NOTNULL(ptcloud);
    CHECK_NOTNULL(colors);
    ptcloud->clear();
    colors->clear();
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);

    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    // Temp variables.
    Color color;
    // Iterate over all blocks.
    for (const BlockIndex& index : blocks) {
        // Iterate over all voxels in said blocks.
        const Block<VoxelType>& block = layer.getBlockByIndex(index);

        for (size_t linear_index = 0; linear_index < num_voxels_per_block;
             ++linear_index) {
            Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
            if (visualizeVoxels(block.getVoxelByLinearIndex(linear_index), &color)) {
                ptcloud->push_back(coord);
                colors->push_back(color);
            }
        }
    }
}

void write_pointcloud_to_file(std::string filepath, Pointcloud *ptcloud, Colors* colors) {
    ofstream myfile;
    myfile.open (filepath);

    for (size_t i = 0; i < ptcloud->size(); ++i) {
        auto position = (*ptcloud)[i];
        auto color = (*colors)[i];
        //myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
        //myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << endl;

        myfile << position(0) << ";" << position(1) << ";" << position(2) << ";";
        myfile << (int) color.r << ";" << (int) color.g << ";" << (int) color.b << ";" << (int) color.a << endl;
    }
    myfile.close();
}



int main(int argc, char** argv) {
    Layer<EsdfVoxel>::Ptr layer_from_file;
    std::string category="obstacles";

    if(argc > 1) {
        category.assign(argv[1]);
    }

    std::string esdf_path="/home/mansoor/esdf_"+category+"_layer.layer";
    std::string pointcloud_path="/home/mansoor/esdf_"+category+"_pointcloud.txt";

    io::LoadLayer<EsdfVoxel>(esdf_path, &layer_from_file);
    Pointcloud ptcloud;
    Colors colors;
    createColorPointcloudFromLayer(*layer_from_file, &ptcloud, &colors);
    write_pointcloud_to_file(pointcloud_path,&ptcloud, &colors);
//    bool save_as_mesh = false;
//    if (save_as_mesh) {
//        std::shared_ptr<MeshLayer> mesh_layer;
//        mesh_layer.reset(new MeshLayer(layer_from_file->block_size()));
//        MeshIntegratorConfig mesh_config;
//        std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator;
//        mesh_integrator.reset(new MeshIntegrator<TsdfVoxel>(mesh_config, *layer_from_file,
//                                                            mesh_layer.get()));
//
//        constexpr bool kOnlyMeshUpdatedBlocks = false;
//        constexpr bool kClearUpdatedFlag = false;
//        mesh_integrator->generateMesh(kOnlyMeshUpdatedBlocks, kClearUpdatedFlag);
//
//        outputMeshLayerAsPly("/home/mansoor/tsdf_layer_obstacles.ply",*mesh_layer);
//    }
    std::cout << "Done!" << std::endl;
    return 0;
}
