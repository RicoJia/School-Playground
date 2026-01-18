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

def run_test(num_topics, message_size="1KB", test_duration=10, use_shm=False, zero_copy=False):
    """
    Run a single test with given parameters.

    Args:
        num_topics (int): Number of topics to test
        message_size (str): Message size ("1KB" or "15KB")
        test_duration (int): How long to run the test in seconds
        use_shm (bool): Whether to use shared memory (True) or UDP (False)
        zero_copy (bool): Whether to enable zero-copy mode (requires use_shm=True)

    Returns:
        tuple: (talker_avg_us, listener_avg_us) or None if failed
    """
    if zero_copy and not use_shm:
        print("Warning: zero_copy requires use_shm=True, ignoring zero_copy flag")
        zero_copy = False
    
    if zero_copy:
        transport = "SHM-ZC"
    elif use_shm:
        transport = "SHM"
    else:
        transport = "UDP"
    
    mode_str = "zero-copy" if zero_copy else "standard"
    print(f"Testing {transport} with {num_topics} topics, {message_size} ({mode_str})...")
    
    # Clean up any lingering processes
    subprocess.run(["pkill", "-9", "-f", "shm_test_many_topics"], capture_output=True)
    time.sleep(2)

    # Setup environment
    env = os.environ.copy()
    if use_shm:
        env['CYCLONEDDS_URI'] = 'file:///root/toolkitt_ws/cyclonedds_shm.xml'
    
    # Output files
    talker_file = f"/tmp/talker_{transport.lower()}_{num_topics}_{message_size}.txt"
    listener_file = f"/tmp/listener_{transport.lower()}_{num_topics}_{message_size}.txt"
    
    # Build command arguments
    zero_copy_arg = str(zero_copy).lower()
    
    try:
        # Start talker
        with open(talker_file, 'w') as tf:
            talker = subprocess.Popen(
                ["bash", "-c", 
                 f"cd /root/toolkitt_ws && source install/setup.bash && "
                 f"ros2 run shm_tests shm_test_many_topics_talker 0 --ros-args "
                 f"--param num_topics:={num_topics} --param message_size:={message_size} "
                 f"--param zero_copy:={zero_copy_arg}"],
                stdout=tf, stderr=subprocess.STDOUT, env=env
            )
                
        # Start listener
        with open(listener_file, 'w') as lf:
            listener = subprocess.Popen(
                ["bash", "-c",
                 f"cd /root/toolkitt_ws && source install/setup.bash && "
                 f"ros2 run shm_tests shm_test_many_topics_listener 0 --ros-args "
                 f"--param num_topics:={num_topics} --param message_size:={message_size} "
                 f"--param zero_copy:={zero_copy_arg}"],
                stdout=lf, stderr=subprocess.STDOUT, env=env
            )
        
        # Wait for test duration
        time.sleep(test_duration)
        
        # Stop processes using pkill (kills actual executables, not wrappers)
        subprocess.run(["pkill", "-TERM", "-f", "shm_test_many_topics_talker"], capture_output=True)
        subprocess.run(["pkill", "-TERM", "-f", "shm_test_many_topics_listener"], capture_output=True)
        
        # Wait for stats output
        time.sleep(5)
        
        # Force kill if still running
        subprocess.run(["pkill", "-9", "-f", "shm_test_many_topics_talker"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "shm_test_many_topics_listener"], capture_output=True)
        
        # Wait for processes to terminate
        talker.wait(timeout=2)
        listener.wait(timeout=2)
        
        # Ensure output is flushed
        time.sleep(1)
        
        # Read output files
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
    
    except Exception as e:
        print(f"  {transport} error: {e}")
        # Cleanup
        subprocess.run(["pkill", "-9", "-f", "shm_test_many_topics"], capture_output=True)
        return None

    # Parse talker output
    talker_avg = None
    for line in reversed(talker_output.split('\n')):
        if 'avg_per_topic=' in line:
            match = re.search(r'avg_per_topic=([\d.]+)', line)
            if match:
                talker_avg = float(match.group(1))
                break

    # Parse listener output
    listener_avg = None
    for line in reversed(listener_output.split('\n')):
        if 'latency' in line.lower() and 'avg=' in line.lower():
            match = re.search(r'avg=([\d.]+)', line, re.IGNORECASE)
            if match:
                listener_avg = float(match.group(1))
                break

    if talker_avg is not None and listener_avg is not None:
        print(f"  {transport} Success: talker_avg={talker_avg:.1f}µs, listener_avg={listener_avg:.1f}µs")
        return talker_avg, listener_avg
    else:
        print(f"  {transport} Failed to parse: talker_avg={talker_avg}, listener_avg={listener_avg}")
        return None

def plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs, 
                    shm_topics, shm_talker_avgs, shm_listener_avgs,
                    shm_zc_topics, shm_zc_talker_avgs, shm_zc_listener_avgs, message_size):
    # Create plot with 2 subplots (simplified version - just talker and listener)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f'Performance vs Number of Topics - {message_size}', fontsize=16, fontweight='bold')

    # Talker comparison
    if udp_talker_avgs:
        ax1.plot(udp_topics, udp_talker_avgs, 'b-o', label='UDP', linewidth=2, markersize=8)
    if shm_talker_avgs:
        ax1.plot(shm_topics, shm_talker_avgs, 'r-s', label='SHM', linewidth=2, markersize=8)
    if shm_zc_talker_avgs:
        ax1.plot(shm_zc_topics, shm_zc_talker_avgs, 'g-^', label='SHM Zero-Copy', linewidth=2, markersize=8)
    
    ax1.set_title('Talker Performance (Publish Time)', fontsize=13, fontweight='bold')
    ax1.set_xlabel('Number of Topics', fontsize=12)
    ax1.set_ylabel('Avg Time per Topic (µs)', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=11)
    
    # Add values on the plot
    for x, y in zip(udp_topics, udp_talker_avgs) if udp_talker_avgs else []:
        ax1.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_topics, shm_talker_avgs) if shm_talker_avgs else []:
        ax1.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_zc_topics, shm_zc_talker_avgs) if shm_zc_talker_avgs else []:
        ax1.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')

    # Listener comparison
    if udp_listener_avgs:
        ax2.plot(udp_topics, udp_listener_avgs, 'b-o', label='UDP', linewidth=2, markersize=8)
    if shm_listener_avgs:
        ax2.plot(shm_topics, shm_listener_avgs, 'r-s', label='SHM', linewidth=2, markersize=8)
    if shm_zc_listener_avgs:
        ax2.plot(shm_zc_topics, shm_zc_listener_avgs, 'g-^', label='SHM Zero-Copy', linewidth=2, markersize=8)
    
    ax2.set_title('Listener Performance (Latency)', fontsize=13, fontweight='bold')
    ax2.set_xlabel('Number of Topics', fontsize=12)
    ax2.set_ylabel('Avg Latency (µs)', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=11)
    
    # Add values on the plot
    for x, y in zip(udp_topics, udp_listener_avgs) if udp_listener_avgs else []:
        ax2.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_topics, shm_listener_avgs) if shm_listener_avgs else []:
        ax2.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_zc_topics, shm_zc_listener_avgs) if shm_zc_listener_avgs else []:
        ax2.text(x, y, f'{y:.1f}', fontsize=9, ha='center', va='bottom')

    plt.tight_layout()
    
    # Save with message size in filename
    plot_filename = f'performance_{message_size.replace(" ", "_")}.png'
    plot_path = f'/root/toolkitt_ws/src/shm_tests/scripts/{plot_filename}'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"  Plot saved: {plot_path}")
    plt.close()

def print_performance(results, header: str):
    # Print summary tables
    print(f"\n=== {header} Results ===")
    if results:
        print("-" * 80)
        print(f"{'Topics':<8} {'Msg Size':<10} {'Talker Avg (µs)':<15} {'Listener Avg (µs)':<18}")
        print("-" * 80)
        for num_topics, msg_size, talker_avg, listener_avg in results:
            print(f"{num_topics:<8} {msg_size:<10} {talker_avg:<15.1f} {listener_avg:<18.1f}")
    else:
        print("No results")

def main():
    """Main testing function."""
    # Test parameters
    topic_counts = [1, 11, 21,31,51]
    message_sizes = ["1KB", "15KB"]
    test_duration = 6.0  # seconds per test

    # Results storage
    udp_results = []
    shm_results = []
    shm_zc_results = []

    print("Starting automated shared memory vs UDP performance testing...")
    print(f"Topic counts: {topic_counts}")
    print(f"Message sizes: {', '.join(message_sizes)}")
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
    time.sleep(4)  # Give RouDi more time to fully initialize and stabilize
    # Verify RouDi is running
    roudi_check = subprocess.run(["pgrep", "-f", "iox-roudi"], capture_output=True)
    if roudi_check.returncode != 0:
        print("ERROR: RouDi failed to start!")
        return
    print("RouDi restarted and verified running.\n")

    # Test each combination of message size and topic count
    for msg_size in message_sizes:
        print(f"\n{'='*70}")
        print(f"=== Testing with {msg_size} messages ===")
        print('='*70)
        
        for num_topics in topic_counts:
            print(f"\n--- Testing with {num_topics} topics ---")
            
            # Test UDP mode
            print(f"  UDP Mode ({msg_size}, {num_topics} topics)...")
            result = run_test(num_topics, msg_size, test_duration, use_shm=False, zero_copy=False)
            if result:
                udp_results.append((num_topics, msg_size, *result))
            
            # Test SHM standard mode
            print(f"  SHM Standard Mode ({msg_size}, {num_topics} topics)...")
            result = run_test(num_topics, msg_size, test_duration, use_shm=True, zero_copy=False)
            if result:
                shm_results.append((num_topics, msg_size, *result))
            
            # Test SHM zero-copy mode
            print(f"  SHM Zero-Copy Mode ({msg_size}, {num_topics} topics)...")
            result = run_test(num_topics, msg_size, test_duration, use_shm=True, zero_copy=True)
            if result:
                shm_zc_results.append((num_topics, msg_size, *result))

    if not udp_results and not shm_results and not shm_zc_results:
        print("No successful tests completed!")
        return

    subprocess.run(["pkill", "-9", "-f", "iox-roudi"], capture_output=True)
    time.sleep(2)
    print("\n" + "="*70)
    print("RouDi stopped.")
    print("="*70)
    
    print_performance(udp_results, "UDP")
    print_performance(shm_results, "SHM Standard")
    print_performance(shm_zc_results, "SHM Zero-Copy")
    
    # Generate plots for each message size
    print("\n" + "="*70)
    print("Generating performance plots...")
    print("="*70)
    
    for msg_size in message_sizes:
        # Filter results for this message size
        udp_data = [(r[0], r[2], r[3]) for r in udp_results if r[1] == msg_size]
        shm_data = [(r[0], r[2], r[3]) for r in shm_results if r[1] == msg_size]
        shm_zc_data = [(r[0], r[2], r[3]) for r in shm_zc_results if r[1] == msg_size]
        
        if not udp_data and not shm_data and not shm_zc_data:
            continue
        
        # Extract data for plotting
        udp_topics = [d[0] for d in udp_data]
        udp_talker_avgs = [d[1] for d in udp_data]
        udp_listener_avgs = [d[2] for d in udp_data]
        
        shm_topics = [d[0] for d in shm_data]
        shm_talker_avgs = [d[1] for d in shm_data]
        shm_listener_avgs = [d[2] for d in shm_data]
        
        shm_zc_topics = [d[0] for d in shm_zc_data]
        shm_zc_talker_avgs = [d[1] for d in shm_zc_data]
        shm_zc_listener_avgs = [d[2] for d in shm_zc_data]
        
        # Generate plot
        plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs,
                        shm_topics, shm_talker_avgs, shm_listener_avgs,
                        shm_zc_topics, shm_zc_talker_avgs, shm_zc_listener_avgs, 
                        f"{msg_size} messages")


if __name__ == "__main__":
    main()
