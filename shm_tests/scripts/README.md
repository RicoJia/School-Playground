# SHM Performance Testing Scripts

This directory contains scripts for automated performance testing of ROS2 shared memory communication.

## shm_performance_test.py

Automated performance testing script that measures latency and throughput for varying numbers of topics with fixed message sizes.

### Usage

```bash
cd /root/toolkitt_ws
source install/setup.bash
python3 shm_tests/scripts/shm_performance_test.py
```

### What it does

1. Tests topic counts from 1 to 100 in steps of 10 (1, 11, 21, ..., 91)
2. Uses fixed 1KB message payloads
3. Runs each configuration for 15 seconds
4. Measures:
   - Talker: Average time per topic for publishing
   - Listener: Average end-to-end latency
5. Generates a performance plot saved to `/root/toolkitt_ws/shm_performance_test.png`
6. Prints a summary table of results

### Output

- Performance plot showing scaling behavior
- Tabular results showing measurements for each topic count
- Console output showing progress and individual test results

### Requirements

- ROS2 Humble with CycloneDDS RMW
- Iceoryx shared memory backend
- matplotlib (for plotting)
- shm_tests package built and installed
