#!/usr/bin/env python3
"""
Parse existing test results from /root/volume/multi-container-shm-test and generate performance plots.
"""

import os
import re
import datetime
import matplotlib.pyplot as plt
import numpy as np

def parse_directory_name(dir_name):
    """Parse directory name to extract test parameters."""
    # Example: 'nodes: 3; topics: 1; msg: 128B; transport: SHM'
    pattern = r'nodes:\s*(\d+);\s*topics:\s*(\d+);\s*msg:\s*([^;]+);\s*transport:\s*(.+)'
    match = re.match(pattern, dir_name)
    if match:
        nodes = int(match.group(1))
        topics = int(match.group(2))
        msg_size = match.group(3).strip()
        transport = match.group(4).strip()
        return nodes, topics, msg_size, transport
    return None

def parse_log_file(filepath):
    """Parse a single log file to extract performance metrics."""
    if not os.path.exists(filepath):
        return None

    with open(filepath, 'r') as f:
        content = f.read()

    # Determine if this is a talker or listener log from filename
    filename = os.path.basename(filepath)
    if 'talker' in filename:
        file_type = 'talker'
    elif 'listener' in filename:
        file_type = 'listener'
    else:
        return None

    if file_type == 'talker':
        # Parse talker output - collect all "avg_per_topic=" values
        avg_values = []
        for line in content.split('\n'):
            if 'avg_per_topic=' in line:
                match = re.search(r'avg_per_topic=([\d.]+)', line)
                if match:
                    avg_values.append(float(match.group(1)))
        
        if avg_values:
            # Take all measurements and let the aggregation function handle outliers
            return avg_values
    else:  # listener
        # Parse listener output - look for latency and message count
        for line in reversed(content.split('\n')):
            if 'latency' in line.lower() and 'avg=' in line.lower():
                # Extract latency
                latency_match = re.search(r'avg=([\d.]+)', line, re.IGNORECASE)
                # Extract message count
                msg_match = re.search(r'total_msgs=(\d+)', line, re.IGNORECASE)
                if latency_match and msg_match:
                    return float(latency_match.group(1)), int(msg_match.group(1))

    return None

def parse_directory_results(dir_path):
    """Parse all log files in a test directory and return aggregated results."""
    all_talker_latencies = []
    all_listener_latencies = []
    all_message_counts = []

    # Find all .log files in the directory
    for filename in os.listdir(dir_path):
        if filename.endswith('.log'):
            filepath = os.path.join(dir_path, filename)
            result = parse_log_file(filepath)

            if result is not None:
                if 'talker' in filename:
                    if isinstance(result, list):
                        # result is a list of all avg_per_topic values from this log
                        all_talker_latencies.extend(result)
                elif 'listener' in filename:
                    if isinstance(result, tuple) and len(result) == 2:
                        latency, msg_count = result
                        all_listener_latencies.append(latency)
                        all_message_counts.append(msg_count)

    # Calculate averages with median for both talkers and listeners to avoid outlier influence
    if all_talker_latencies and all_listener_latencies and all_message_counts:
        # Use median for talker
        all_talker_latencies.sort()
        n_talker = len(all_talker_latencies)
        if n_talker % 2 == 0:
            avg_talker = (all_talker_latencies[n_talker//2 - 1] + all_talker_latencies[n_talker//2]) / 2
        else:
            avg_talker = all_talker_latencies[n_talker//2]
        
        # Use median for listener
        all_listener_latencies.sort()
        n_listener = len(all_listener_latencies)
        if n_listener % 2 == 0:
            avg_listener = (all_listener_latencies[n_listener//2 - 1] + all_listener_latencies[n_listener//2]) / 2
        else:
            avg_listener = all_listener_latencies[n_listener//2]
        
        total_msgs = sum(all_message_counts)
        return avg_talker, avg_listener, total_msgs

    return None

def plot_performance(udp_topics, udp_talker_avgs, udp_listener_avgs, udp_msg_counts,
                     shm_topics, shm_talker_avgs, shm_listener_avgs, shm_msg_counts,
                     shm_zc_topics, shm_zc_talker_avgs, shm_zc_listener_avgs, shm_zc_msg_counts, message_size):
    """Generate performance comparison plots."""
    # Create plot with 3 subplots (talker, listener, and message count)
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle(f'Performance vs Total Number of Topics - {message_size} Messages', fontsize=16, fontweight='bold')

    # Talker comparison
    if udp_talker_avgs:
        ax1.plot(udp_topics, udp_talker_avgs, 'b-o', label='UDP', linewidth=2, markersize=8)
    if shm_talker_avgs:
        ax1.plot(shm_topics, shm_talker_avgs, 'r-s', label='SHM', linewidth=2, markersize=8)
    if shm_zc_talker_avgs:
        ax1.plot(shm_zc_topics, shm_zc_talker_avgs, 'g-^', label='SHM Zero-Copy', linewidth=2, markersize=8)

    ax1.set_title('Talker Performance (Median Publish Time)', fontsize=13, fontweight='bold')
    ax1.set_xlabel('Total Number of Topics', fontsize=12)
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

    ax2.set_title('Listener Performance (Message Delivery Time)', fontsize=13, fontweight='bold')
    ax2.set_xlabel('Total Number of Topics', fontsize=12)
    ax2.set_ylabel('Message Delivery Time (µs)', fontsize=12)
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
    ax3.set_xlabel('Total Number of Topics', fontsize=12)
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

    # Save with timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
    plot_filename = f'parsed-performance-results_{message_size.replace(" ", "_")}_{timestamp}.png'
    plot_path = f'/tmp/{plot_filename}'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved: {plot_path}")
    plt.close()

def main():
    """Main function to parse results and generate plots."""
    base_dir = "/root/volume/multi-container-shm-test"

    if not os.path.exists(base_dir):
        print(f"Error: Directory {base_dir} does not exist")
        return

    # Data structures to collect results by message size
    results_by_msg_size = {}  # msg_size -> {'UDP': [], 'SHM': [], 'SHM-zero-copy': []}

    print("Parsing test results from existing directories...")

    # Parse each directory
    for dirname in os.listdir(base_dir):
        dir_path = os.path.join(base_dir, dirname)

        if not os.path.isdir(dir_path):
            continue

        # Parse directory name
        params = parse_directory_name(dirname)
        if not params:
            print(f"Warning: Could not parse directory name: {dirname}")
            continue

        nodes, topics, msg_size, transport = params

        # Initialize data structure for this message size if not exists
        if msg_size not in results_by_msg_size:
            results_by_msg_size[msg_size] = {
                'UDP': {'topics': [], 'talker_avgs': [], 'listener_avgs': [], 'msg_counts': []},
                'SHM': {'topics': [], 'talker_avgs': [], 'listener_avgs': [], 'msg_counts': []},
                'SHM-zero-copy': {'topics': [], 'talker_avgs': [], 'listener_avgs': [], 'msg_counts': []}
            }

        print(f"Parsing {dirname}...")
        print(f"  Parameters: {nodes} nodes, {topics} topics, {msg_size}, {transport}")

        # Parse the results
        result = parse_directory_results(dir_path)
        if result:
            avg_talker, avg_listener, total_msgs = result
            total_topics = int(topics * nodes)  # Calculate total topics in system
            print(f"  Results: talker={avg_talker:.1f}µs, listener={avg_listener:.1f}µs, msgs={total_msgs}")

            # Add to appropriate data structure
            transport_key = transport
            results_by_msg_size[msg_size][transport_key]['topics'].append(total_topics)
            results_by_msg_size[msg_size][transport_key]['talker_avgs'].append(avg_talker)
            results_by_msg_size[msg_size][transport_key]['listener_avgs'].append(avg_listener)
            results_by_msg_size[msg_size][transport_key]['msg_counts'].append(total_msgs)
        else:
            print(f"  Warning: Could not parse results from {dirname}")

    # Generate plots for each message size
    print(f"\nGenerating performance plots for {len(results_by_msg_size)} message sizes...")
    
    for msg_size, transport_data in results_by_msg_size.items():
        print(f"\nProcessing {msg_size} messages...")
        
        # Extract data for this message size
        udp_data = transport_data['UDP']
        shm_data = transport_data['SHM']
        shm_zc_data = transport_data['SHM-zero-copy']
        
        # Sort the data by topics for each transport
        def sort_transport_data(data_dict):
            if data_dict['topics']:
                combined = list(zip(data_dict['topics'], data_dict['talker_avgs'], 
                                  data_dict['listener_avgs'], data_dict['msg_counts']))
                combined.sort(key=lambda x: x[0])
                data_dict['topics'], data_dict['talker_avgs'], data_dict['listener_avgs'], data_dict['msg_counts'] = zip(*combined)
        
        sort_transport_data(udp_data)
        sort_transport_data(shm_data)
        sort_transport_data(shm_zc_data)
        
        # Generate plot for this message size
        plot_performance(udp_data['topics'], udp_data['talker_avgs'], udp_data['listener_avgs'], udp_data['msg_counts'],
                        shm_data['topics'], shm_data['talker_avgs'], shm_data['listener_avgs'], shm_data['msg_counts'],
                        shm_zc_data['topics'], shm_zc_data['talker_avgs'], shm_zc_data['listener_avgs'], shm_zc_data['msg_counts'],
                        msg_size)

if __name__ == "__main__":
    main()
