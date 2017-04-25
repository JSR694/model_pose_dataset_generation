#!/usr/bin/env sh

# Credit: https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKDIR="$( rospack find model_pose_dataset_generation )"

source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$PACKDIR
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PACKDIR/models

# Optionally, include bigbird models dir.
# See included script for setting those up.

if [ -d ~/.gazebo/models/bigbird ]; then
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models/bigbird
fi

# And just in case the default dir isn't there...
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
