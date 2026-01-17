#!/bin/bash
cd /root/toolkitt_ws
source install/setup.bash
iox-roudi &
sleep 2
export CYCLONEDDS_URI=file:///root/toolkitt_ws/cyclonedds_shm.xml

echo "Starting talker..."
ros2 run shm_tests shm_test_many_topics_talker 0 --ros-args --param num_topics:=1 --param message_length:=1024 > /tmp/shm_talker_test.log 2>&1 &
TALKER_PID=$!
echo "Talker PID: $TALKER_PID"

sleep 2

echo "Starting listener..."
ros2 run shm_tests shm_test_many_topics_listener 0 --ros-args --param num_topics:=1 --param message_length:=1024 > /tmp/shm_listener_test.log 2>&1 &
LISTENER_PID=$!
echo "Listener PID: $LISTENER_PID"

sleep 10

echo "Stopping processes..."
kill $TALKER_PID $LISTENER_PID $(pgrep -f iox-roudi) 2>/dev/null
timeout 5s wait $TALKER_PID $LISTENER_PID $(pgrep -f iox-roudi) 2>/dev/null || true
kill -9 $TALKER_PID $LISTENER_PID $(pgrep -f iox-roudi) 2>/dev/null || true
wait

echo ""
echo "=== Talker Output (last 10 lines) ==="
tail -10 /tmp/shm_talker_test.log

echo ""
echo "=== Listener Output (last 10 lines) ==="
tail -10 /tmp/shm_listener_test.log

echo ""
echo "=== Checking for latency data ==="
grep "Latency:" /tmp/shm_listener_test.log | tail -1
