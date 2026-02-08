#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def load_and_plot(filename='time_delay_data.json'):
    """Load data and create visualization plots"""

    # Load data
    with open(filename, 'r') as f:
        data = json.load(f)

    camera_data = data['camera_data']
    imu_data = data['imu_data']

    # Calculate delays
    camera_delays = []
    camera_times = []
    for item in camera_data:
        if item['clock_time_at_reception'] is not None:
            delay = item['clock_time_at_reception'] - item['msg_time']
            camera_delays.append(delay * 1000)  # Convert to ms
            camera_times.append(item['wall_time'])

    imu_delays = []
    imu_times = []
    for item in imu_data:
        if item['clock_time_at_reception'] is not None:
            delay = item['clock_time_at_reception'] - item['msg_time']
            imu_delays.append(delay * 1000)  # Convert to ms
            imu_times.append(item['wall_time'])

    # Normalize times to start from 0
    if camera_times:
        start_time = min(camera_times[0] if camera_times else float('inf'),
                        imu_times[0] if imu_times else float('inf'))
        camera_times = [(t - start_time) for t in camera_times]
        imu_times = [(t - start_time) for t in imu_times]

    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Time Delay Analysis: Camera vs IMU vs /clock', fontsize=16, fontweight='bold')

    # Plot 1: Time delays over time
    ax1 = axes[0]
    if camera_delays:
        ax1.plot(camera_times, camera_delays, 'b.-', alpha=0.6, label='Camera', markersize=3)
    if imu_delays:
        ax1.plot(imu_times, imu_delays, 'r.-', alpha=0.6, label='IMU', markersize=3)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Delay (ms)')
    ax1.set_title('Time Delay vs Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Histogram of delays
    ax2 = axes[1]
    if camera_delays:
        ax2.hist(camera_delays, bins=50, alpha=0.6, label='Camera', color='blue', edgecolor='black')
    if imu_delays:
        ax2.hist(imu_delays, bins=50, alpha=0.6, label='IMU', color='red', edgecolor='black')

    ax2.set_xlabel('Delay (ms)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Delay Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: Delay difference (Camera - IMU)
    ax3 = axes[2]

    # Interpolate to common time points for comparison
    if camera_delays and imu_delays:
        # Find common time range
        min_time = max(min(camera_times), min(imu_times))
        max_time = min(max(camera_times), max(imu_times))

        # Show the absolute difference
        cam_mean = np.mean(camera_delays)
        imu_mean = np.mean(imu_delays)

        ax3.axhline(y=cam_mean, color='blue', linestyle='--', label=f'Camera Mean: {cam_mean:.3f} ms')
        ax3.axhline(y=imu_mean, color='red', linestyle='--', label=f'IMU Mean: {imu_mean:.3f} ms')
        ax3.axhline(y=cam_mean - imu_mean, color='green', linestyle='-', linewidth=2,
                   label=f'Difference: {cam_mean - imu_mean:.3f} ms')

        ax3.set_xlabel('Delay (ms)')
        ax3.set_ylabel('Value (ms)')
        ax3.set_title('Mean Delay Comparison')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_xlim(-max(abs(cam_mean), abs(imu_mean)) * 2, max(abs(cam_mean), abs(imu_mean)) * 2)

    plt.tight_layout()

    # Save figure
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_file = f'time_delay_analysis_{timestamp}.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f'\nPlot saved to: {output_file}')

    # Print statistics
    print('\n' + '='*80)
    print('STATISTICAL SUMMARY')
    print('='*80)

    if camera_delays:
        print('\nCAMERA Statistics:')
        print(f'  Mean:   {np.mean(camera_delays):.6f} ms')
        print(f'  Median: {np.median(camera_delays):.6f} ms')
        print(f'  Std:    {np.std(camera_delays):.6f} ms')
        print(f'  Min:    {np.min(camera_delays):.6f} ms')
        print(f'  Max:    {np.max(camera_delays):.6f} ms')
        print(f'  Range:  {np.max(camera_delays) - np.min(camera_delays):.6f} ms')

        # Coefficient of variation (normalized measure of variability)
        cv = (np.std(camera_delays) / abs(np.mean(camera_delays))) * 100 if np.mean(camera_delays) != 0 else 0
        print(f'  Coefficient of Variation: {cv:.2f}%')

    if imu_delays:
        print('\nIMU Statistics:')
        print(f'  Mean:   {np.mean(imu_delays):.6f} ms')
        print(f'  Median: {np.median(imu_delays):.6f} ms')
        print(f'  Std:    {np.std(imu_delays):.6f} ms')
        print(f'  Min:    {np.min(imu_delays):.6f} ms')
        print(f'  Max:    {np.max(imu_delays):.6f} ms')
        print(f'  Range:  {np.max(imu_delays) - np.min(imu_delays):.6f} ms')

        cv = (np.std(imu_delays) / abs(np.mean(imu_delays))) * 100 if np.mean(imu_delays) != 0 else 0
        print(f'  Coefficient of Variation: {cv:.2f}%')

    if camera_delays and imu_delays:
        print('\nCOMPARISON:')
        print(f'  Mean difference (Camera - IMU): {np.mean(camera_delays) - np.mean(imu_delays):.6f} ms')
        print(f'  Std difference:  {abs(np.std(camera_delays) - np.std(imu_delays)):.6f} ms')

    print('='*80)

    plt.show()


if __name__ == '__main__':
    import sys

    filename = sys.argv[1] if len(sys.argv) > 1 else 'time_delay_data.json'

    try:
        load_and_plot(filename)
    except FileNotFoundError:
        print(f'Error: File {filename} not found.')
        print('Please run time_delay_analyzer.py first to collect data.')
    except Exception as e:
        print(f'Error: {e}')
