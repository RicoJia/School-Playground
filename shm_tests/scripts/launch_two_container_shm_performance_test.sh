#!/bin/bash

# Launch script for two-container SHM performance test
# Runs all permutations of options for the specified mode
# Usage: ./launch_two_container_shm_performance_test.sh {listener|talker}

# Color definitions
GREEN='\033[0;32m'
NC='\033[0m'  # No Color

function _print_header(){
    local MESSAGE=$1
    local width=40
    local green="${GREEN}"
    local nc="${NC}"
    # Print top line
    printf "${green}%${width}s${nc}\n" | tr ' ' '='
    # Print centered message
    printf "${green}%*s${nc}\n" $(( (width + ${#MESSAGE}) / 2 )) "$MESSAGE"
    # Print bottom line
    printf "${green}%${width}s${nc}\n" | tr ' ' '='
    echo ""
}

MODE=$1

if [[ "$MODE" != "listener" && "$MODE" != "talker" ]]; then
    echo "Usage: $0 {listener|talker}"
    echo "This script runs all permutations of test options for the specified mode."
    exit 1
fi

# Define option arrays
NUM_NODES_OPTIONS=(1 5 10 15)  # Can add more: (1 2 3)
NUM_TOPICS_OPTIONS=(1) # Can add more: (1 2)
MSG_KB_OPTIONS=("128B" "1KB" "15KB")  #
TRANSPORT_OPTIONS=("UDP" "SHM" "SHM-zero-copy") # 

_print_header "Starting iox-roudi"

# Start iox-roudi if not already running
echo "Checking iox-roudi status..."
if pgrep -x "iox-roudi" > /dev/null; then
    echo "iox-roudi is already running"
else
    echo "Starting iox-roudi..."
    export LD_LIBRARY_PATH=/root/toolkitt_ws/install/iceoryx_posh/lib:/root/toolkitt_ws/install/iceoryx_binding_c/lib:$LD_LIBRARY_PATH
    iox-roudi &
    IOX_ROUDI_PID=$!
    echo "iox-roudi started with PID: $IOX_ROUDI_PID"
    # Give it a moment to start up
    sleep 2
fi

echo "Running all permutations for $MODE mode..."

# Record start time
START_TIME=$(date +%s)

# Loop through all combinations
for num_nodes in "${NUM_NODES_OPTIONS[@]}"; do
    for num_topics in "${NUM_TOPICS_OPTIONS[@]}"; do
        for msg_kb in "${MSG_KB_OPTIONS[@]}"; do
            for transport_variant in "${TRANSPORT_OPTIONS[@]}"; do
                run_id="nodes: ${num_nodes}; topics: ${num_topics}; msg: ${msg_kb}; transport: ${transport_variant}"
                
                _print_header "$run_id"
                
                echo "Running: mode=$MODE, nodes=$num_nodes, topics=$num_topics, msg=$msg_kb, transport_variant=$transport_variant, run_id=$run_id"
                
                # Run the command
                ros2 run shm_tests two_container_shm_performance_test.py \
                    --mode "$MODE" \
                    --num-nodes "$num_nodes" \
                    --num-topics "$num_topics" \
                    --msg-kb "$msg_kb" \
                    --transport-variant "$transport_variant" \
                    --run-id "$run_id"
                
                echo "Completed run: $run_id"
                echo "----------------------------------------"
            done
        done
    done
done

echo "All permutations completed for $MODE mode."

# Calculate and display total test time
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))
echo "Total test time: $DURATION seconds"

# Clean up iox-roudi if we started it
if [[ -n "$IOX_ROUDI_PID" ]]; then
    echo "Stopping iox-roudi (PID: $IOX_ROUDI_PID)..."
    kill $IOX_ROUDI_PID 2>/dev/null || true
    wait $IOX_ROUDI_PID 2>/dev/null || true
    echo "iox-roudi stopped"
fi