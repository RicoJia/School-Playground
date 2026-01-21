#!/usr/bin/env python3
"""
Automated performance testing script for shared memory ROS2 nodes.
Tests varying numbers of topics with fixed 1KB message size.
Compares UDP vs Shared Memory performance.
"""

import signal
import subprocess
import time
import re
import datetime
import matplotlib.pyplot as plt
import os
import sys

TOPICS_PER_NODE = 3      # Topics per node

def _start_processes(num_nodes, num_TOPICS_PER_NODE, message_size, use_shm, zero_copy):
    """
    Start the talker and listener processes for testing.
    
    Returns:
        tuple: (talker_procs, listener_procs, output_files, transport)
    """
    if zero_copy:
        transport = "SHM-ZC"
    elif use_shm:
        transport = "SHM"
    else:
        transport = "UDP"
    
    # Setup environment
    env = os.environ.copy()
    if use_shm:
        env['CYCLONEDDS_URI'] = 'file:///root/toolkitt_ws/src/shm_tests/src/shm_tests/configs/cyclonedds_shm.xml'
    
    # Build command arguments
    zero_copy_arg = str(zero_copy).lower()
    
    talker_procs = []
    listener_procs = []
    output_files = []
    # NOTE: Removed the global "pkill -9 -f shm_test_many_topics" cleanup.
    # If you want a cleanup, do it ONCE at program start, and prefer SIGTERM, not SIGKILL.

    def _spawn(role: str, node_id: int):
        outfile = f"/tmp/{role}_{transport.lower()}_node{node_id}.txt"
        output_files.append((role, outfile))
        f = open(outfile, "w")

        # Keep bash -lc because you need `source ...setup.bash`.
        # `-l` (login shell) is optional; `-c` runs the command.
        cmd = (
            "cd /root/toolkitt_ws && "
            "source install/setup.bash && "
            f"ros2 run shm_tests shm_test_many_topics_{role} {node_id} --ros-args "
            f"--param num_topics:={num_TOPICS_PER_NODE} "
            f"--param message_size:={message_size} "
            f"--param zero_copy:={zero_copy_arg}"
        )

        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=f,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,   # NEW process group/session; proc.pid == pgid
        )
        return proc, f, outfile
 
    # Start listeners
    for node_id in range(num_nodes):
        proc, f, _ = _spawn("listener", node_id)
        listener_procs.append((proc, f))


    # Give listeners time to start
    time.sleep(2)
    
    for node_id in range(num_nodes):
        proc, f, _ = _spawn("talker", node_id)
        talker_procs.append((proc, f))

    return talker_procs, listener_procs, output_files, transport

def _kill_processes(talker_procs, listener_procs, term_timeout=5.0, kill_timeout=2.0):
    """
    Kill the running processes gracefully.
    """
    procs = talker_procs + listener_procs
    for proc, f in procs:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            # process already killed
            pass
    
    # 2) Reap anything that exits during the grace window
    deadline = time.time() + term_timeout
    alive = []
    for proc, _f in procs:
        remaining = max(0.0, deadline - time.time())
        if remaining == 0.0:
            alive.append((proc, _f))
            continue
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            alive.append((proc, _f))

    # 3) Only SIGKILL the stubborn ones
    for proc, _f in alive:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    # 4) Final reap + close files
    for proc, f in procs:
        try:
            proc.wait(timeout=kill_timeout)
        except subprocess.TimeoutExpired:
            print(f"WARNING: process group {proc.pid} did not terminate in time")
        finally:
            try:
                f.flush()
            except Exception:
                pass
            f.close()


def _parse_file_outputs(output_files, transport, num_nodes):
    """
    Parse output files from talker and listener processes.
    
    Returns:
        tuple: (avg_talker_latency_us, avg_listener_latency_us, total_msg_count) or None if failed
    """
    # Parse all output files
    talker_avgs = []
    listener_avgs = []
    listener_msg_counts = []
    
    for file_type, filepath in output_files:
        if not os.path.exists(filepath):
            print(f"  Warning: {filepath} not found")
            continue
            
        with open(filepath, 'r') as f:
            content = f.read()
        
        found_stats = False
        if file_type == 'talker':
            # Parse talker output
            for line in reversed(content.split('\n')):
                if 'avg_per_topic=' in line:
                    match = re.search(r'avg_per_topic=([\d.]+)', line)
                    if match:
                        talker_avgs.append(float(match.group(1)))
                        found_stats = True
                        break
        else:  # listener
            # Parse listener output
            for line in reversed(content.split('\n')):
                if 'latency' in line.lower() and 'avg=' in line.lower():
                    # Extract latency
                    match = re.search(r'avg=([\d.]+)', line, re.IGNORECASE)
                    if match:
                        listener_avgs.append(float(match.group(1)))
                    # Extract message count
                    msg_match = re.search(r'total_msgs=(\d+)', line, re.IGNORECASE)
                    if msg_match:
                        listener_msg_counts.append(int(msg_match.group(1)))
                        found_stats = True
                        break
        
        # If stats not found, show the output for debugging
        if not found_stats:
            print(f"  {transport} {file_type} output from {os.path.basename(filepath)}:")
            # Show last 20 lines or all if less
            lines = content.split('\n')
            for line in lines[-30:]:
                if line.strip():
                    print(f"    {line}")
        
        # Clean up output file
        # os.remove(filepath)
        print(f"Tmp file path: {filepath}")

    # Calculate averages across all nodes
    if talker_avgs and listener_avgs and listener_msg_counts:
        avg_talker = sum(talker_avgs) / len(talker_avgs)
        avg_listener = sum(listener_avgs) / len(listener_avgs)
        total_msgs = sum(listener_msg_counts)
        print(f"  {transport} Success: {num_nodes} nodes, talker_avg={avg_talker:.1f}µs, listener_avg={avg_listener:.1f}µs, total_msgs={total_msgs}")
        print(f"    Per-node talker: {talker_avgs}")
        print(f"    Per-node listener: {listener_avgs}")
        print(f"    Per-node msg counts: {listener_msg_counts}")
        return avg_talker, avg_listener, total_msgs
    else:
        print(f"  {transport} Failed to parse: talker_count={len(talker_avgs)}, listener_count={len(listener_avgs)}, msg_count={len(listener_msg_counts)}")
        return None

def run_test(num_nodes, num_TOPICS_PER_NODE, message_size="1KB", test_duration=10.0, use_shm=False, zero_copy=False):
    """
    Run a test with multiple nodes, each handling multiple topics.

    Args:
        num_nodes (int): Number of talker/listener node pairs to spawn
        num_TOPICS_PER_NODE (int): Number of topics per node
        message_size (str): Message size ("1KB" or "15KB")
        test_duration (float): How long to run the test in seconds
        use_shm (bool): Whether to use shared memory (True) or UDP (False)
        zero_copy (bool): Whether to enable zero-copy mode (requires use_shm=True)

    Returns:
        tuple: (avg_talker_latency_us, avg_listener_latency_us, total_msg_count) or None if failed
    """
    if not use_shm:
        zero_copy = False
    
    mode_str = "zero-copy" if zero_copy else "standard"
    
    talker_procs, listener_procs, output_files, transport = _start_processes(num_nodes, num_TOPICS_PER_NODE, message_size, use_shm, zero_copy)
    
    print(f"Testing {transport}: {num_nodes} nodes × {num_TOPICS_PER_NODE} topics/node, {message_size} ({mode_str})...")
    
    # Wait for test duration
    time.sleep(test_duration)
    
    _kill_processes(talker_procs, listener_procs)
    
    time.sleep(5)
    
    return _parse_file_outputs(output_files, transport, num_nodes)

def plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs, udp_msg_counts,
                    shm_topics, shm_talker_avgs, shm_listener_avgs, shm_msg_counts,
                    shm_zc_topics, shm_zc_talker_avgs, shm_zc_listener_avgs, shm_zc_msg_counts, message_size):
    # Create plot with 3 subplots (talker, listener, and message count)
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle(f'Performance vs Number of Topics ({TOPICS_PER_NODE} per node) - {message_size}', fontsize=16, fontweight='bold')

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

    # Message count comparison
    if udp_msg_counts:
        ax3.plot(udp_topics, udp_msg_counts, 'b-o', label='UDP', linewidth=2, markersize=8)
    if shm_msg_counts:
        ax3.plot(shm_topics, shm_msg_counts, 'r-s', label='SHM', linewidth=2, markersize=8)
    if shm_zc_msg_counts:
        ax3.plot(shm_zc_topics, shm_zc_msg_counts, 'g-^', label='SHM Zero-Copy', linewidth=2, markersize=8)
    
    ax3.set_title('Messages Received', fontsize=13, fontweight='bold')
    ax3.set_xlabel('Number of Topics', fontsize=12)
    ax3.set_ylabel('Total Messages Received', fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=11)
    
    # Add values on the plot
    for x, y in zip(udp_topics, udp_msg_counts) if udp_msg_counts else []:
        ax3.text(x, y, f'{y}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_topics, shm_msg_counts) if shm_msg_counts else []:
        ax3.text(x, y, f'{y}', fontsize=9, ha='center', va='bottom')
    for x, y in zip(shm_zc_topics, shm_zc_msg_counts) if shm_zc_msg_counts else []:
        ax3.text(x, y, f'{y}', fontsize=9, ha='center', va='bottom')

    plt.tight_layout()
    
    # Save with message size in filename
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
    plot_filename = f'multi-node-performance_{message_size.replace(" ", "_")}_{timestamp}.png'
    plot_path = f'/tmp/{plot_filename}'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"  Plot saved: {plot_path}")
    plt.close()

def print_performance(results, header: str):
    # Print summary tables
    print(f"\n=== {header} Results ===")
    if results:
        print("-" * 95)
        print(f"{'Topics':<8} {'Msg Size':<10} {'Talker Avg (µs)':<15} {'Listener Avg (µs)':<18} {'Total Messages Received':<15}")
        print("-" * 95)
        for num_topics, msg_size, talker_avg, listener_avg, msg_count in results:
            print(f"{num_topics:<8} {msg_size:<10} {talker_avg:<15.1f} {listener_avg:<18.1f} {msg_count:<15}")
    else:
        print("No results")

def main():
    """Main testing function."""
    # Test parameters
    node_counts = [1, 5, 10, 15, 20]# Number of talker/listener pairs
    message_sizes = ["128B", "1KB", "15KB"]  # Message sizes to test

    # node_counts = [1]# Number of talker/listener pairs
    # message_sizes = ["1KB"]  # Message sizes to test

    test_duration = 30.0      # seconds per test

    # Results storage
    udp_results = []
    shm_results = []
    shm_zc_results = []

    print("Starting automated shared memory vs UDP performance testing...")
    print(f"Node counts: {node_counts}")
    print(f"Topics per node: {TOPICS_PER_NODE}")
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

    # Test each combination of message size and node count
    for msg_size in message_sizes:
        print(f"\n{'='*70}")
        print(f"=== Testing with {msg_size} messages ===")
        print('='*70)
        
        for num_nodes in node_counts:
            print(f"\n--- Testing with {num_nodes} nodes × {TOPICS_PER_NODE} topics/node ---")
            total_topics = num_nodes * TOPICS_PER_NODE
            
            # Test UDP mode
            print(f"  UDP Mode ({msg_size}, {num_nodes} nodes)...")
            result = run_test(num_nodes, TOPICS_PER_NODE, msg_size, test_duration, use_shm=False, zero_copy=False)
            if result:
                udp_results.append((total_topics, msg_size, *result))  # (topics, msg_size, talker_avg, listener_avg, msg_count)
            
            # Test SHM standard mode
            print(f"  SHM Standard Mode ({msg_size}, {num_nodes} nodes)...")
            result = run_test(num_nodes, TOPICS_PER_NODE, msg_size, test_duration, use_shm=True, zero_copy=False)
            if result:
                shm_results.append((total_topics, msg_size, *result))  # (topics, msg_size, talker_avg, listener_avg, msg_count)
            
            # Test SHM zero-copy mode
            print(f"  SHM Zero-Copy Mode ({msg_size}, {num_nodes} nodes)...")
            result = run_test(num_nodes, TOPICS_PER_NODE, msg_size, test_duration, use_shm=True, zero_copy=True)
            if result:
                shm_zc_results.append((total_topics, msg_size, *result))  # (topics, msg_size, talker_avg, listener_avg, msg_count)

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
        udp_data = [(r[0], r[2], r[3], r[4]) for r in udp_results if r[1] == msg_size]
        shm_data = [(r[0], r[2], r[3], r[4]) for r in shm_results if r[1] == msg_size]
        shm_zc_data = [(r[0], r[2], r[3], r[4]) for r in shm_zc_results if r[1] == msg_size]
        
        if not udp_data and not shm_data and not shm_zc_data:
            continue
        
        # Extract data for plotting
        udp_topics = [d[0] for d in udp_data]
        udp_talker_avgs = [d[1] for d in udp_data]
        udp_listener_avgs = [d[2] for d in udp_data]
        udp_msg_counts = [d[3] for d in udp_data]
        
        shm_topics = [d[0] for d in shm_data]
        shm_talker_avgs = [d[1] for d in shm_data]
        shm_listener_avgs = [d[2] for d in shm_data]
        shm_msg_counts = [d[3] for d in shm_data]
        
        shm_zc_topics = [d[0] for d in shm_zc_data]
        shm_zc_talker_avgs = [d[1] for d in shm_zc_data]
        shm_zc_listener_avgs = [d[2] for d in shm_zc_data]
        shm_zc_msg_counts = [d[3] for d in shm_zc_data]
        
        # Generate plot
        plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs, udp_msg_counts,
                        shm_topics, shm_talker_avgs, shm_listener_avgs, shm_msg_counts,
                        shm_zc_topics, shm_zc_talker_avgs, shm_zc_listener_avgs, shm_zc_msg_counts,
                        f"{msg_size} messages")


if __name__ == "__main__":
    main()
