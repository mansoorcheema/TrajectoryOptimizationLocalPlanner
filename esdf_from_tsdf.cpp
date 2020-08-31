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

int main() {
    Layer<TsdfVoxel>::Ptr layer_from_file;
    io::LoadLayer<TsdfVoxel>("/home/mansoor/tsdf_free_layer.layer", &layer_from_file);

    //save esdf layer
    // ESDF maps.
    EsdfMap::Config esdf_config;
    // Same number of voxels per side for ESDF as with TSDF
    esdf_config.esdf_voxels_per_side = layer_from_file->voxels_per_side();
    // Same voxel size for ESDF as with TSDF
    esdf_config.esdf_voxel_size = layer_from_file->voxel_size();

    EsdfIntegrator::Config esdf_integrator_config;

    EsdfMap esdf_map(esdf_config);
    EsdfIntegrator esdf_integrator(esdf_integrator_config, layer_from_file.get(),
                                   esdf_map.getEsdfLayerPtr());

    esdf_integrator.updateFromTsdfLayerBatch();

    const bool esdf_success = io::SaveLayer(esdf_map.getEsdfLayer(), "/home/mansoor/esdf_free_layer.layer");

    if (esdf_success == false) {
        throw std::runtime_error("Failed to save ESDF");
    }
    std::cout << "Save Esdf from tsdf!" << std::endl;
    return 0;
}
