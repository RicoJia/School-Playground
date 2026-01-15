#!/bin/bash

# Script to rebuild Iceoryx and dependent packages with custom limits
# Usage: ./rebuild_iceoryx_limits.sh [publishers] [subscribers] [notifiers]
# Default: 2048 for all limits

set -e  # Exit on error

# Default values
PUBLISHERS=${1:-2048}
SUBSCRIBERS=${2:-2048}
NOTIFIERS=${3:-2048}

WORKSPACE_ROOT="$HOME/toolkitt_ws"
ICEORYX_DIR="$WORKSPACE_ROOT/iceoryx"
DEPLOYMENT_FILE="$ICEORYX_DIR/iceoryx_posh/cmake/IceoryxPoshDeployment.cmake"

echo "=================================================="
echo "Rebuilding Iceoryx with custom limits"
echo "  Publishers: $PUBLISHERS"
echo "  Subscribers: $SUBSCRIBERS"
echo "  Notifiers: $NOTIFIERS"
echo "=================================================="

# Step 0: Clone repositories if they don't exist
echo ""
echo "[0/7] Checking/cloning required repositories..."
cd "$WORKSPACE_ROOT"

# Clone Iceoryx if not present
if [ ! -d "$ICEORYX_DIR" ]; then
    echo "  Cloning iceoryx..."
    git clone https://github.com/eclipse-iceoryx/iceoryx.git
    cd "$ICEORYX_DIR"
    git checkout v2.0.5
else
    echo "  iceoryx already exists"
fi

# Clone CycloneDDS if not present
if [ ! -d "$WORKSPACE_ROOT/src/cyclonedds" ]; then
    echo "  Cloning cyclonedds..."
    mkdir -p "$WORKSPACE_ROOT/src"
    cd "$WORKSPACE_ROOT/src"
    git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
    cd cyclonedds
    git checkout releases/0.10.x
else
    echo "  cyclonedds already exists"
fi

# Clone rmw_cyclonedds if not present
if [ ! -d "$WORKSPACE_ROOT/src/rmw_cyclonedds" ]; then
    echo "  Cloning rmw_cyclonedds..."
    cd "$WORKSPACE_ROOT/src"
    git clone https://github.com/ros2/rmw_cyclonedds.git
    cd rmw_cyclonedds
    git checkout humble
else
    echo "  rmw_cyclonedds already exists"
fi

# Step 1: Update the deployment configuration
echo ""
echo "[1/7] Updating Iceoryx deployment configuration..."
if [ ! -f "$DEPLOYMENT_FILE" ]; then
    echo "Error: Deployment file not found at $DEPLOYMENT_FILE"
    exit 1
fi

# Create backup
cp "$DEPLOYMENT_FILE" "$DEPLOYMENT_FILE.bak"

# Update the three limit values using sed
sed -i "s/set(IOX_MAX_PUBLISHERS [0-9]\+)/set(IOX_MAX_PUBLISHERS $PUBLISHERS)/" "$DEPLOYMENT_FILE"
sed -i "s/set(IOX_MAX_SUBSCRIBERS [0-9]\+)/set(IOX_MAX_SUBSCRIBERS $SUBSCRIBERS)/" "$DEPLOYMENT_FILE"
sed -i "s/set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS [0-9]\+)/set(IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS $NOTIFIERS)/" "$DEPLOYMENT_FILE"

echo "  Updated limits in $DEPLOYMENT_FILE"
echo "  Verifying changes:"
grep "IOX_MAX_PUBLISHERS\|IOX_MAX_SUBSCRIBERS\|IOX_INTERNAL_MAX_NUMBER_OF_NOTIFIERS" "$DEPLOYMENT_FILE" | grep "^set"

# Step 2: Clean old build artifacts
echo ""
echo "[2/7] Cleaning old build artifacts..."
cd "$WORKSPACE_ROOT"
rm -rf build/iceoryx_posh build/iceoryx_binding_c build/iceoryx_hoofs build/iceoryx_platform
rm -rf install/iceoryx_posh install/iceoryx_binding_c install/iceoryx_hoofs install/iceoryx_platform
rm -rf build/cyclonedds install/cyclonedds
rm -rf build/rmw_cyclonedds install/rmw_cyclonedds
echo "  Cleaned build and install directories"

# Step 3: Build Iceoryx packages
echo ""
echo "[3/7] Building Iceoryx packages..."
cd "$WORKSPACE_ROOT"
colcon build --packages-select iceoryx_platform iceoryx_hoofs iceoryx_posh iceoryx_binding_c \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "  Iceoryx build complete"

# Step 4: Build CycloneDDS with shared memory support
echo ""
echo "[4/7] Building CycloneDDS with shared memory support..."
source "$WORKSPACE_ROOT/install/setup.bash"
colcon build --packages-select cyclonedds \
    --cmake-args -DENABLE_SHM=ON -DCMAKE_BUILD_TYPE=Release

echo "  CycloneDDS build complete"

# Step 5: Build rmw_cyclonedds_cpp
echo ""
echo "[5/7] Building rmw_cyclonedds_cpp..."
source "$WORKSPACE_ROOT/install/setup.bash"
colcon build --packages-select rmw_cyclonedds_cpp \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "  rmw_cyclonedds_cpp build complete"

# Step 6: Rebuild shm_tests
echo ""
echo "[6/7] Rebuilding shm_tests..."
source "$WORKSPACE_ROOT/install/setup.bash"

# Handle both possible locations
if [ -d "$WORKSPACE_ROOT/shm_tests" ]; then
    rm -rf build/shm_tests install/shm_tests
    colcon build --packages-select shm_tests
elif [ -d "$WORKSPACE_ROOT/src/shm_tests" ]; then
    rm -rf build/shm_tests install/shm_tests
    colcon build --packages-select shm_tests
else
    echo "  Warning: shm_tests package not found"
fi

echo "  shm_tests build complete"

# Step 7: Verify library linkage
echo ""
echo "[7/7] Verifying library linkage..."
if [ -f "$WORKSPACE_ROOT/install/rmw_cyclonedds_cpp/lib/librmw_cyclonedds_cpp.so" ]; then
    echo "  Checking rmw_cyclonedds_cpp dependencies:"
    ldd "$WORKSPACE_ROOT/install/rmw_cyclonedds_cpp/lib/librmw_cyclonedds_cpp.so" | grep iceoryx || echo "  No iceoryx dependencies (might use static linking)"
fi

echo ""
echo "=================================================="
echo "Build complete!"
echo "=================================================="
echo ""
echo "Next steps:"
echo "1. Source the workspace:"
echo "   source $WORKSPACE_ROOT/install/setup.bash"
echo ""
echo "2. Restart iox-roudi with the new build:"
echo "   pkill iox-roudi"
echo "   iox-roudi"
echo ""
echo "3. To enable shared memory in ROS2, set:"
echo "   export CYCLONEDDS_URI=file:///tmp/cyclonedds_shm.xml"
echo ""
echo "4. Run your nodes:"
echo "   ros2 run shm_tests shm_test_many_topics_talker 0"
echo "   ros2 run shm_tests shm_test_many_topics_listener 0"
echo ""
