# LiDAR Anomaly Detection

This directory contains scripts for detecting anomalies in LiDAR data, specifically occlusions versus out-of-range readings.

## anomally_detection.py

A Python script that analyzes LiDAR scan data from CSV files to detect and classify occlusions using temporal analysis and segment stability tracking.

### Features

- **Occlusion Detection**: Distinguishes between true occlusions and out-of-range sensor readings
- **Temporal Analysis**: Uses historical data to improve detection accuracy
- **Segment Tracking**: Groups adjacent blocked rays into coherent occlusion segments
- **Stability Scoring**: Measures how consistent occlusions are over time
- **Configurable Parameters**: Adjustable thresholds for different environments

### Input Format

CSV files with LiDAR scan data in the format:
```
timestep,lidar_0,lidar_1,lidar_2,...,lidar_359
1,3.45,3.22,3.18,...,inf
2,3.44,inf,inf,...,3.67
...
```

### Output Format

CSV files with occlusion annotations:
```
timestep,has_occlusion,num_segments,angle_ranges_deg,beam_indices,stabilities
1,1,2,"-45.0 to -30.0; 120.0 to 135.0","15-30; 240-255","0.85; 0.92"
2,0,0,"","",""
...
```

### Usage

```bash
python anomally_detection.py \
    --input lidar_data.csv \
    --output occlusions.csv \
    --history-size 30 \
    --min-segment-beams 5 \
    --persistence-threshold 0.7 \
    --min-occlusion-width-deg 5.0
```

### Parameters

- `--input`: Input CSV file with LiDAR scan data
- `--output`: Output CSV file for occlusion annotations
- `--history-size`: Number of previous scans to consider (default: 30)
- `--min-segment-beams`: Minimum rays needed to form an occlusion segment (default: 5)
- `--gap-merge-beams`: Maximum gap between rays to merge into one segment (default: 2)
- `--drift-tolerance-beams`: Tolerance for segment position changes (default: 3)
- `--persistence-threshold`: Minimum stability score for valid occlusion (default: 0.7)
- `--min-occlusion-width-deg`: Minimum angular width for occlusions in degrees (default: 5.0)
- `--angle-min-deg`: Starting angle of lidar_0 in degrees (default: -180.0)
- `--angle-span-deg`: Total angular coverage in degrees (default: 360.0)

### Algorithm Overview

1. **Infinite Value Detection**: Identifies rays with infinite/max range readings
2. **Segment Formation**: Groups adjacent infinite rays into potential occlusion segments
3. **Historical Tracking**: Compares current segments with previous scans
4. **Stability Analysis**: Calculates how consistently each segment appears over time
5. **Classification**: Determines if segments represent true occlusions based on stability

### Dependencies

- Python 3.6+
- Standard library only (csv, math, argparse, collections, dataclasses, typing)

### Example

```bash
# Analyze LiDAR data with default parameters
python anomally_detection.py --input lidar_data_1.csv --output occlusions_1.csv

# Use custom parameters for noisy environment
python anomally_detection.py \
    --input lidar_data.csv \
    --output occlusions.csv \
    --history-size 50 \
    --persistence-threshold 0.8 \
    --min-occlusion-width-deg 10.0
```

This tool is particularly useful for robotics applications where distinguishing between sensor limitations and actual obstacles is critical for navigation and mapping.


# ROS Bag to CSV Converter

## About

The `turtlebot_bag_converter.py` script converts ROS bag files containing LiDAR data to CSV format compatible with the TurtleBot LiDAR Occlusion Simulator.

## What it does

- **Input**: ROS 2 bag files (.db3) with LiDAR scan data
- **Output**: CSV files with format `timestep,lidar_0,lidar_1,...,lidar_359`
- **Processing**: Automatically interpolates LiDAR data to exactly 360 points (0° to 359°)
- **Compatibility**: Matches the format used in your existing datasets

## Usage

```bash
# Navigate to the directory
cd /home/user13/Turtlebot_LidarOcclusion_Simulator/real_robot/

# Convert your bag files
python3 turtlebot_bag_converter.py ros_bags/run1/ -o run1_lidar_data.csv
python3 turtlebot_bag_converter.py ros_bags/run2/ -o run2_lidar_data.csv
python3 turtlebot_bag_converter.py ros_bags/run3/ -o run3_lidar_data.csv
```

## Output Format

Each CSV file contains:
- **Header**: `timestep,lidar_0,lidar_1,...,lidar_359`
- **Data**: Each row = one complete 360° LiDAR scan
- **Values**: Range measurements in meters (or 'inf' for no detection)

## Results

- **run1**: 596 scans
- **run2**: 584 scans  
- **run3**: 358 scans