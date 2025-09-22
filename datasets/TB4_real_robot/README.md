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