#!/bin/bash

# trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"
# echo "Script directory is: ${WORKSPACE_DIR}"


gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                  roslaunch pcd_manage pcd_rviz.launch ; exec bash"
