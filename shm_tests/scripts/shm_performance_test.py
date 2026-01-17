#!/usr/bin/env python3
"""
Automated performance testing script for shared memory ROS2 nodes.
Tests varying numbers of topics with fixed 1KB message size.
Compares UDP vs Shared Memory performance.
"""

import subprocess
import time
import re
import matplotlib.pyplot as plt
import os
import sys

def run_test(num_topics, message_length=1024, test_duration=10, use_shm=False):
    """
    Run a single test with given parameters.

    Args:
        num_topics (int): Number of topics to test
        message_length (int): Size of message payload in bytes
        test_duration (int): How long to run the test in seconds
        use_shm (bool): Whether to use shared memory (True) or UDP (False)

    Returns:
        tuple: (talker_avg_us, listener_avg_us) or None if failed
    """
    transport = "SHM" if use_shm else "UDP"
    print(f"Testing {transport} with {num_topics} topics, {message_length} bytes...")
    
    # Clean up any lingering processes
    subprocess.run(["pkill", "-9", "-f", "shm_test_many_topics"], capture_output=True)
    time.sleep(2)

    # Create a test script with proper environment variable handling
    env_line = ""
    if use_shm:
        env_line = "export CYCLONEDDS_URI=file:///root/toolkitt_ws/cyclonedds_shm.xml"
    
    script_content = f"""#!/bin/bash
cd /root/toolkitt_ws
source install/setup.bash
{env_line}

# Start talker in background
ros2 run shm_tests shm_test_many_topics_talker 0 --ros-args --param num_topics:={num_topics} --param message_length:={message_length} > /tmp/talker_{transport.lower()}_{num_topics}.txt 2>&1 &
TALKER_PID=$!

# Wait a bit for talker to start
sleep 2

# Start listener in background
ros2 run shm_tests shm_test_many_topics_listener 0 --ros-args --param num_topics:={num_topics} --param message_length:={message_length} > /tmp/listener_{transport.lower()}_{num_topics}.txt 2>&1 &
LISTENER_PID=$!

# Run for specified duration
sleep {test_duration}

# Stop processes gracefully
kill $TALKER_PID $LISTENER_PID 2>/dev/null
sleep 2
kill -9 $TALKER_PID $LISTENER_PID 2>/dev/null
wait $TALKER_PID 2>/dev/null
wait $LISTENER_PID 2>/dev/null

# Extra cleanup
pkill -9 -f "many_topics_talker.*--param num_topics:={num_topics}" 2>/dev/null
pkill -9 -f "many_topics_listener.*--param num_topics:={num_topics}" 2>/dev/null
sleep 2
"""

    # Write and run the script
    script_path = f"/tmp/test_script_{transport.lower()}_{num_topics}.sh"
    with open(script_path, 'w') as f:
        f.write(script_content)

    os.chmod(script_path, 0o755)

    try:
        result = subprocess.run([script_path], capture_output=True, text=True, timeout=test_duration + 10)

        # Read the output files
        talker_file = f"/tmp/talker_{transport.lower()}_{num_topics}.txt"
        listener_file = f"/tmp/listener_{transport.lower()}_{num_topics}.txt"

        talker_output = ""
        listener_output = ""

        if os.path.exists(talker_file):
            with open(talker_file, 'r') as f:
                talker_output = f.read()
            os.remove(talker_file)

        if os.path.exists(listener_file):
            with open(listener_file, 'r') as f:
                listener_output = f.read()
            os.remove(listener_file)

        # Clean up script
        if os.path.exists(script_path):
            os.remove(script_path)

    except subprocess.TimeoutExpired:
        print(f"  {transport} test timed out")
        return None
    except Exception as e:
        print(f"  {transport} error: {e}")
        return None

    # Parse talker output - get last avg_per_topic measurement
    talker_avg = None
    talker_lines = talker_output.strip().split('\n')
    for line in reversed(talker_lines):
        if 'avg_per_topic=' in line:
            match = re.search(r'avg_per_topic=([\d.]+)', line)
            if match:
                talker_avg = float(match.group(1))
                break

    # Parse listener output - get last avg measurement
    listener_avg = None
    listener_lines = listener_output.strip().split('\n')
    for line in reversed(listener_lines):
        if 'Latency:' in line and 'avg=' in line:
            avg_match = re.search(r'avg=([\d.]+)', line)
            if avg_match:
                listener_avg = float(avg_match.group(1))
                break

    if talker_avg is not None and listener_avg is not None:
        print(f"  {transport} Success: talker_avg={talker_avg:.1f}µs, listener_avg={listener_avg:.1f}µs")
        return talker_avg, listener_avg
    else:
        print(f"  {transport} Failed to parse: talker_avg={talker_avg}, listener_avg={listener_avg}")
        if use_shm:
            print(f"  Talker output lines: {len(talker_lines)}, Listener output lines: {len(listener_lines)}")
        return None

def plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs, 
                    shm_topics, shm_talker_avgs, shm_listener_avgs, message_length):
    # Create plot
    plt.figure(figsize=(14, 10))

    # Talker comparison
    plt.subplot(2, 2, 1)
    if udp_talker_avgs:
        plt.plot(udp_topics, udp_talker_avgs, 'b-o', label='UDP', linewidth=2, markersize=6)
    if shm_talker_avgs:
        plt.plot(shm_topics, shm_talker_avgs, 'r-s', label='SHM', linewidth=2, markersize=6)
    plt.title(f'Talker Performance - Message Size: {message_length} bytes')
    plt.xlabel('Number of Topics')
    plt.ylabel('Avg Time per Topic (µs)')
    plt.grid(True, alpha=0.3)
    plt.legend()

    # Listener comparison
    plt.subplot(2, 2, 2)
    if udp_listener_avgs:
        plt.plot(udp_topics, udp_listener_avgs, 'b-o', label='UDP', linewidth=2, markersize=6)
    if shm_listener_avgs:
        plt.plot(shm_topics, shm_listener_avgs, 'r-s', label='SHM', linewidth=2, markersize=6)
    plt.title(f'Listener Latency - Message Size: {message_length} bytes')
    plt.xlabel('Number of Topics')
    plt.ylabel('Avg Latency (µs)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig('/root/toolkitt_ws/src/shm_tests/scripts/shm_vs_udp_performance.png', dpi=300, bbox_inches='tight')
    print(f"\nPlot saved to: /root/toolkitt_ws/src/shm_tests/scripts/shm_vs_udp_performance.png")

def print_performance(udp_results, header: str):
    # Print summary tables
    print(f"\n=== {header} Results ===")
    if udp_results:
        print("-" * 70)
        print(f"{'Topics':<8} {'Talker Avg (µs)':<15} {'Listener Avg (µs)':<18}")
        print("-" * 70)
        for num_topics, talker_avg, listener_avg in udp_results:
            print(f"{num_topics:<8} {talker_avg:<15.1f} {listener_avg:<18.1f}")
    else:
        print("No UDP results")

def main():
    """Main testing function."""
    # Test parameters
    num_topics_range = range(1, 102, 10)  # 1, 11, 21, ..., 91
    message_length = 1024 * 150  # 1KB
    test_duration = 15  # seconds per test

    # Results storage
    udp_results = []
    shm_results = []

    print("Starting automated shared memory vs UDP performance testing...")
    print(f"Message size: {message_length} bytes")
    print(f"Test duration per configuration: {test_duration} seconds")
    print("-" * 70)

    # Restart roudi for fresh SHM environment
    print("\n=== Restarting RouDi for SHM tests ===")
    subprocess.run(["pkill", "-9", "-f", "iox-roudi"], capture_output=True)
    time.sleep(3)
    roudi_proc = subprocess.Popen(
        ["bash", "-c", "cd /root/toolkitt_ws && source install/setup.bash && iox-roudi -l off"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    # time.sleep(3)
    print("RouDi restarted.\n")

    # Test UDP mode
    print("\n=== Testing UDP Mode ===")
    for num_topics in num_topics_range:
        result = run_test(num_topics, message_length, test_duration, use_shm=False)
        if result:
            udp_results.append((num_topics, *result))
    

    # Test SHM mode
    print("=== Testing SHM Mode ===")
    for num_topics in num_topics_range:
        result = run_test(num_topics, message_length, test_duration, use_shm=True)
        if result:
            shm_results.append((num_topics, *result))

    if not udp_results and not shm_results:
        print("No successful tests completed!")
        return

    subprocess.run(["pkill", "-9", "-f", "iox-roudi"], capture_output=True)
    time.sleep(2)
    print("RouDi stopped.\n")
    
    # Extract data for plotting
    udp_topics = [r[0] for r in udp_results] if udp_results else []
    udp_talker_avgs = [r[1] for r in udp_results] if udp_results else []
    udp_listener_avgs = [r[2] for r in udp_results] if udp_results else []

    shm_topics = [r[0] for r in shm_results] if shm_results else []
    shm_talker_avgs = [r[1] for r in shm_results] if shm_results else []
    shm_listener_avgs = [r[2] for r in shm_results] if shm_results else []

    # Generate plots
    plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs,
                    shm_topics, shm_talker_avgs, shm_listener_avgs, message_length)

    print_performance(udp_results, "UDP")
    print_performance(shm_results, "SHM")


if __name__ == "__main__":
    main()
