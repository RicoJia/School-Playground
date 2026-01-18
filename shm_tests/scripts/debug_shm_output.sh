#!/bin/bash
cd /root/toolkitt_ws
source install/setup.bash
export CYCLONEDDS_URI=file:///root/toolkitt_ws/cyclonedds_shm.xml

# Ensure RouDi is running for SHM
if ! pgrep -f "iox-roudi" > /dev/null; then
    echo "Starting RouDi..."
    iox-roudi -l off > /tmp/roudi.log 2>&1 &
    sleep 3
fi

echo "Starting talker..."
ros2 run shm_tests shm_test_many_topics_talker 0 --ros-args --param num_topics:=1 --param message_length:=1024 > /tmp/debug_talker.txt 2>&1 &
TALKER_PID=$!

echo "Starting listener..."
ros2 run shm_tests shm_test_many_topics_listener 0 --ros-args --param num_topics:=1 --param message_length:=1024 > /tmp/debug_listener.txt 2>&1 &
LISTENER_PID=$!

sleep 8

echo "Stopping processes..."
# Kill the actual executables, not just the wrapper scripts
# Find and kill the actual talker/listener processes
pkill -TERM -f "shm_test_many_topics_talker.*"
pkill -TERM -f "shm_test_many_topics_listener.*"

# Wait for listener to process SIGTERM and print stats
sleep 5

# Force kill if still running
pkill -9 -f "shm_test_many_topics_talker.*"
pkill -9 -f "shm_test_many_topics_listener.*"

# Wait for processes to terminate
wait $TALKER_PID 2>/dev/null
wait $LISTENER_PID 2>/dev/null

# Ensure output is flushed to files
sync
sleep 2

echo ""
echo "=== Full Talker Output ==="
if [ -f /tmp/debug_talker.txt ]; then
    cat /tmp/debug_talker.txt
else
    echo "Talker output file not found"
fi
echo ""
echo "=== Full Listener Output ==="
if [ -f /tmp/debug_listener.txt ]; then
    cat /tmp/debug_listener.txt
else
    echo "Listener output file not found"
fi
