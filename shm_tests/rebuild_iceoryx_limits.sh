#!/bin/bash
# Script to update Iceoryx limits and rebuild affected packages
# Usage example: ~/toolkitt_ws/rebuild_iceoryx_limits.sh 4096 4096 4096

set -e  # Exit on error

# Configuration - modify these values as needed
NEW_MAX_PUBLISHERS=${1:-2048}
NEW_MAX_SUBSCRIBERS=${2:-2048}
NEW_MAX_NOTIFIERS=${3:-2048}

WORKSPACE_DIR="${HOME}/toolkitt_ws"
DEPLOYMENT_FILE="${WORKSPACE_DIR}/iceoryx/iceoryx_posh/cmake/IceoryxPoshDeployment.cmake"

echo "=================================================="
echo "Iceoryx Limits Update and Rebuild Script"
echo "=================================================="
echo "Target limits:"
echo "  IOX_MAX_PUBLISHERS: ${NEW_MAX_PUBLISHERS}"
echo "  IOX_MAX_SUBSCRIBERS: ${NEW_MAX_SUBSCRIBERS}"
echo "  IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS: ${NEW_MAX_NOTIFIERS}"
echo "=================================================="

# Check if deployment file exists
if [ ! -f "${DEPLOYMENT_FILE}" ]; then
    echo "ERROR: Deployment file not found: ${DEPLOYMENT_FILE}"
    echo "Please ensure Iceoryx source is cloned at ${WORKSPACE_DIR}/iceoryx"
    exit 1
fi

# Step 1: Update the deployment configuration file
echo ""
echo "[1/3] Updating Iceoryx deployment configuration..."
cd "${WORKSPACE_DIR}/iceoryx"

sed -i "s/set(IOX_MAX_PUBLISHERS [0-9]\+)/set(IOX_MAX_PUBLISHERS ${NEW_MAX_PUBLISHERS})/" \
    iceoryx_posh/cmake/IceoryxPoshDeployment.cmake

sed -i "s/set(IOX_MAX_SUBSCRIBERS [0-9]\+)/set(IOX_MAX_SUBSCRIBERS ${NEW_MAX_SUBSCRIBERS})/" \
    iceoryx_posh/cmake/IceoryxPoshDeployment.cmake

sed -i "s/set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS [0-9]\+)/set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS ${NEW_MAX_NOTIFIERS})/" \
    iceoryx_posh/cmake/IceoryxPoshDeployment.cmake

echo "✓ Configuration updated:"
grep "IOX_MAX_PUBLISHERS\|IOX_MAX_SUBSCRIBERS\|IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS" \
    iceoryx_posh/cmake/IceoryxPoshDeployment.cmake | grep "set(" | sed 's/^/  /'

# Step 2: Clean build and install directories for affected packages
echo ""
echo "[2/3] Cleaning build artifacts for affected packages..."
cd "${WORKSPACE_DIR}"
rm -rf build/iceoryx_posh build/iceoryx_binding_c
rm -rf install/iceoryx_posh install/iceoryx_binding_c
echo "✓ Clean complete"

# Step 3: Rebuild affected packages
echo ""
echo "[3/3] Building Iceoryx packages with new limits..."
cd "${WORKSPACE_DIR}"
colcon build --packages-select iceoryx_posh iceoryx_binding_c \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo ""
echo "=================================================="
echo "✓ Build complete!"
echo "=================================================="
echo "Next steps:"
echo "  1. Source the workspace: source ~/toolkitt_ws/install/setup.bash"
echo "  2. Restart iox-roudi daemon"
echo "  3. Run your ROS2 nodes"
echo "=================================================="
