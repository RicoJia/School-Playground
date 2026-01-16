#!/usr/bin/env python3
"""
Automated performance testing script for shared memory ROS2 nodes.
Tests varying numbers of topics with fixed 1KB message size.
"""

import subprocess
import time
import re
import matplotlib.pyplot as plt
import os
import sys

def run_test(num_topics, message_length=1024, test_duration=10):
    """
    Run a single test with given parameters.

    Args:
        num_topics (int): Number of topics to test
        message_length (int): Size of message payload in bytes
        test_duration (int): How long to run the test in seconds

    Returns:
        tuple: (talker_avg_us, listener_avg_us) or None if failed
    """
    print(f"Testing with {num_topics} topics, {message_length} bytes...")

    # Create a test script
    script_content = f"""#!/bin/bash
cd /root/toolkitt_ws
source install/setup.bash

# Start talker in background
ros2 run shm_tests shm_test_many_topics_talker 0 --ros-args --param num_topics:={num_topics} --param message_length:={message_length} > /tmp/talker_output_{num_topics}.txt 2>&1 &
TALKER_PID=$!

# Wait a bit for talker to start
sleep 2

# Start listener in background
ros2 run shm_tests shm_test_many_topics_listener 0 --ros-args --param num_topics:={num_topics} --param message_length:={message_length} > /tmp/listener_output_{num_topics}.txt 2>&1 &
LISTENER_PID=$!

# Run for specified duration
sleep {test_duration}

# Stop processes
kill $TALKER_PID $LISTENER_PID 2>/dev/null
wait $TALKER_PID 2>/dev/null
wait $LISTENER_PID 2>/dev/null
"""

    # Write and run the script
    script_path = f"/tmp/test_script_{num_topics}.sh"
    with open(script_path, 'w') as f:
        f.write(script_content)

    os.chmod(script_path, 0o755)

    try:
        result = subprocess.run([script_path], capture_output=True, text=True, timeout=test_duration + 10)

        # Read the output files
        talker_file = f"/tmp/talker_output_{num_topics}.txt"
        listener_file = f"/tmp/listener_output_{num_topics}.txt"

        if os.path.exists(talker_file):
            with open(talker_file, 'r') as f:
                talker_output = f.read()
        else:
            talker_output = ""

        if os.path.exists(listener_file):
            with open(listener_file, 'r') as f:
                listener_output = f.read()
        else:
            listener_output = ""

        # Clean up
        os.remove(script_path)
        if os.path.exists(talker_file):
            os.remove(talker_file)
        if os.path.exists(listener_file):
            os.remove(listener_file)

    except subprocess.TimeoutExpired:
        print("  Test timed out")
        return None
    except Exception as e:
        print(f"  Error during test: {e}")
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
        if 'Latency:' in line:
            avg_match = re.search(r'avg=([\d.]+)', line)
            if avg_match:
                listener_avg = float(avg_match.group(1))
            break

    if talker_avg is not None and listener_avg is not None:
        print(f"  Success: talker_avg={talker_avg:.1f}µs, listener_avg={listener_avg:.1f}µs")
        return talker_avg, listener_avg
    else:
        print(f"  Failed to parse measurements (talker_avg: {talker_avg}, listener_avg: {listener_avg})")
        return None

def main():
    """Main testing function."""
    # Test parameters
    num_topics_range = range(1, 101, 10)  # 1, 11, 21, ..., 91
    message_length = 1024  # 1KB
    test_duration = 15  # seconds per test

    # Results storage
    results = []

    print("Starting automated shared memory performance testing...")
    print(f"Message size: {message_length} bytes")
    print(f"Test duration per configuration: {test_duration} seconds")
    print("-" * 60)

    for num_topics in num_topics_range:
        result = run_test(num_topics, message_length, test_duration)
        if result:
            results.append((num_topics, *result))

    if not results:
        print("No successful tests completed!")
        return

    # Extract data for plotting
    topics = [r[0] for r in results]
    talker_avgs = [r[1] for r in results]
    listener_avgs = [r[2] for r in results]

    # Create plot
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(topics, talker_avgs, 'b-o', label='Talker avg_per_topic (µs)', linewidth=2, markersize=6)
    plt.title(f'Shared Memory Performance - Message Size: {message_length} bytes')
    plt.xlabel('Number of Topics')
    plt.ylabel('Time (µs)')
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(topics, listener_avgs, 'r-s', label='Listener avg latency (µs)', linewidth=2, markersize=6)
    plt.xlabel('Number of Topics')
    plt.ylabel('Latency (µs)')
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.savefig('/root/toolkitt_ws/shm_performance_test.png', dpi=300, bbox_inches='tight')
    print(f"\nPlot saved to: /root/toolkitt_ws/shm_performance_test.png")

    # Print summary table
    print("\nSummary Results:")
    print("-" * 70)
    print(f"{'Topics':<8} {'Talker Avg (µs)':<15} {'Listener Avg (µs)':<18}")
    print("-" * 70)
    for num_topics, talker_avg, listener_avg in results:
        print(f"{num_topics:<8} {talker_avg:<15.1f} {listener_avg:<18.1f}")

    plt.show()

if __name__ == "__main__":
    main()
