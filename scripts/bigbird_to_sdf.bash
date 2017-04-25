#!/usr/bin/env bash

# Converts a bigbird model at the provided dir to a proper SDF that can e.g. be read by gazebo.

if [ "$#" -ne 2 ]; then
        echo "Usage: bigbird_to_sdf MODELS_DIR MODEL_NAME"
        exit 1
fi


modelsdir=$1
modelname=$2

startdir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "dest $modelsdir"
cd $modelsdir/$modelname
echo $PWD

mkdir -p materials/textures
mkdir meshes

# Move and rename texture:
mv optimized_tsdf_texture_mapped_mesh.png materials/textures/${modelname}.png

# Update texture location:
sed -i -e "s/optimized_tsdf.*.png/..\/materials\/textures\/${modelname}.png/g" optimized_tsdf_texture_mapped_mesh_BOTTOMFIX.dae 

# Move and rename mesh:
mv optimized_tsdf_texture_mapped_mesh_BOTTOMFIX.dae meshes/${modelname}.dae

# Update mesh location:
sed -i -e "s/optimized_tsdf.*.dae/meshes\/${modelname}.dae/g" model.sdf
sed -i -e "s/optimized_tsdf.*.dae/meshes\/${modelname}.dae/g" model-1_4.sdf

# remove any references to old sdf versions in config:
sed -i -e 's/<sdf.*version.*1.2.*sdf>//g' model.config
sed -i -e 's/<sdf.*version.*1.3.*sdf>//g' model.config

cd $startdir
