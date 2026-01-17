# SHM Performance Testing Scripts

This directory contains scripts for automated performance testing of ROS2 shared memory communication.

## shm_performance_test.py

Automated performance testing script that measures latency and throughput for varying numbers of topics with fixed message sizes. Compares UDP vs Shared Memory performance.

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
4. Tests both transport modes:
   - **UDP Mode**: Standard CycloneDDS UDP transport
   - **SHM Mode**: Shared memory transport via Iceoryx (requires roudi daemon)
5. Measures:
   - Talker: Average time per topic for publishing
   - Listener: Average end-to-end latency
6. Generates comparative performance plots saved to `/root/toolkitt_ws/shm_vs_udp_performance.png`
7. Prints summary tables for both transport modes

### Prerequisites for SHM Mode

1. **Iceoryx RouDi daemon** must be running:
   ```bash
   cd /root/toolkitt_ws && source install/setup.bash
   iox-roudi
   ```

2. **CycloneDDS configuration** for shared memory:
   - Configuration file: `/root/toolkitt_ws/cyclonedds_shm.xml`
   - Contains: `<SharedMemory><Enable>true</Enable></SharedMemory>`

### Output

- **Comparative plots**: 4-panel plot showing UDP vs SHM performance
- **Tabular results**: Separate tables for UDP and SHM measurements
- **Console output**: Progress and individual test results

### Expected Results

- **UDP Mode**: Baseline performance with network transport
- **SHM Mode**: Typically lower latency and higher throughput for local communication
- **Scaling**: Both modes show increasing latency with more topics, but SHM should scale better

### Requirements

- ROS2 Humble with CycloneDDS RMW
- Iceoryx shared memory backend (for SHM mode)
- matplotlib (for plotting)
- shm_tests package built and installed
