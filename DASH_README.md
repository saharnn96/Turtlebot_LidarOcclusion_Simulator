# TurtleBot LiDAR Occlusion Simulator - Dash Version

## Overview

This is a modern web-based version of the TurtleBot LiDAR Occlusion Simulator built with Dash and Plotly. It provides an interactive dashboard that can be accessed through any web browser.

## Features

### 🌐 Web-Based Interface

- **Modern UI**: Bootstrap-based responsive design
- **Real-time Updates**: Live dashboard with automatic refresh
- **Cross-Platform**: Access from any device with a web browser
- **Interactive Plots**: Click-to-navigate on the map

### 🤖 Robot Simulation

- **Real-time Position Tracking**: Live robot position and heading
- **LiDAR Simulation**: Normal and occluded sensor data
- **Dual Navigation Modes**: Standard and spin configuration
- **Obstacle Avoidance**: A\* pathfinding algorithm

### 📊 Visualization

- **Interactive Map**: Real-time robot position, trajectory, and obstacles
- **Polar LiDAR Plot**: Live sensor data visualization
- **Status Indicators**: Real-time system status badges
- **Control Panel**: Easy-to-use control buttons

## Installation

1. **Install Dependencies**:

   ```bash
   pip install -r requirements.txt
   ```

2. **Install Dash-specific packages** (if not included above):
   ```bash
   pip install dash>=2.14.0 dash-bootstrap-components>=1.4.0 plotly>=5.15.0
   ```

## Usage

### Running the Dash App

1. **Start the Application**:

   ```bash
   python dash_turtlebotsim.py
   ```

2. **Access the Dashboard**:
   - Open your web browser
   - Navigate to: `http://localhost:8050`
   - The dashboard will load automatically

### Interface Components

#### 📋 Control Panel (Left Column)

- **Status Indicators**: Real-time LiDAR and navigation mode status
- **Control Buttons**:
  - Toggle LiDAR Occlusion
  - Toggle Navigation Mode (Standard/Spin Config)
  - Stop Navigation
- **Manual Navigation**: Input target coordinates manually
- **Robot Status**: Live position, heading, and system information

#### 🗺️ Map Visualization (Center Column)

- **Interactive Map**: Click anywhere to navigate the robot
- **Robot Position**: Blue circle with heading arrow
- **Trajectory**: Green dashed line showing planned path
- **Obstacles**: Red X markers showing static obstacles
- **Real-time Updates**: Position updates every second

#### 📡 LiDAR Visualization (Right Column)

- **Polar Plot**: 360-degree LiDAR data visualization
- **Range Data**: Distance measurements in all directions
- **Occlusion Simulation**: Shows infinite readings when occluded

### Navigation Methods

#### 1. **Click Navigation**

- Click anywhere on the map
- Robot automatically calculates path using A\* algorithm
- Navigation starts immediately

#### 2. **Manual Coordinate Input**

- Enter X and Y coordinates in the control panel
- Click "Navigate to Target" button
- Robot moves to specified position

#### 3. **MQTT Command Control**

- Send commands via MQTT/Redis
- Supports external navigation systems
- Compatible with ROS2 nav2_simple_commander

### Configuration

#### Navigation Modes

- **Standard Mode**: Direct movement to target
- **Spin Config Mode**: Includes rotation movements during navigation

#### LiDAR Simulation

- **Normal Mode**: Random distance readings (5-10 units)
- **Occluded Mode**: First 300 degrees show infinite readings

## Architecture

### Key Components

#### `TurtleBotSim` Class

```python
class TurtleBotSim:
    def __init__(self):
        self.position = [0, 0]          # Robot position [x, y]
        self.heading = 0                # Robot orientation (radians)
        self.lidar_occluded = False     # LiDAR occlusion state
        self.standard_navigation = True  # Navigation mode
        # ... other properties
```

#### Dash Callbacks

- **Plot Updates**: Real-time visualization refresh
- **Button Handlers**: Control panel interactions
- **Map Clicks**: Click-to-navigate functionality
- **Status Updates**: Live system status indicators

#### Communication

- **MQTT/Redis**: External command interface
- **JSON Messaging**: Standardized message format
- **Threading**: Non-blocking navigation execution

## Comparison with Tkinter Version

| Feature               | Tkinter Version    | Dash Version              |
| --------------------- | ------------------ | ------------------------- |
| **Interface**         | Desktop GUI        | Web Browser               |
| **Accessibility**     | Local only         | Network accessible        |
| **Responsiveness**    | Platform-dependent | Cross-platform            |
| **Interactivity**     | Click events       | Click + hover + zoom      |
| **Styling**           | Basic              | Modern Bootstrap UI       |
| **Real-time Updates** | Manual refresh     | Automatic refresh         |
| **Multi-user**        | Single user        | Multiple browser sessions |

## API Endpoints

When running, the Dash app provides:

- **Main Dashboard**: `http://localhost:8050/`
- **JSON API**: Access to plot data via Dash's internal API
- **Callbacks**: Real-time updates via WebSocket connections

## Troubleshooting

### Common Issues

1. **Port Already in Use**:

   ```bash
   # Change port in the code
   app.run_server(debug=True, host='0.0.0.0', port=8051)
   ```

2. **Dependencies Missing**:

   ```bash
   pip install dash dash-bootstrap-components plotly
   ```

3. **MQTT Connection Issues**:

   - Check Redis server is running
   - Verify connection parameters
   - Check firewall settings

4. **Browser Compatibility**:
   - Use modern browsers (Chrome, Firefox, Safari, Edge)
   - Enable JavaScript
   - Clear browser cache if needed

### Performance Tips

- **Update Interval**: Adjust `interval` in `dcc.Interval` for different refresh rates
- **Data Limits**: Limit trajectory history for better performance
- **Browser Resources**: Close unused browser tabs

## Development

### Adding New Features

1. **New Callbacks**:

   ```python
   @app.callback(
       Output('component-id', 'property'),
       [Input('trigger-id', 'property')]
   )
   def callback_function(trigger_value):
       # Your logic here
       return new_value
   ```

2. **New Components**:

   ```python
   # Add to layout
   dbc.Button("New Feature", id="new-btn", color="primary")
   ```

3. **Styling**:
   - Use Bootstrap classes for consistent styling
   - Custom CSS can be added via `assets/` folder

### Deployment

#### Local Network Access

```python
app.run_server(debug=False, host='0.0.0.0', port=8050)
```

#### Production Deployment

- Use Gunicorn or similar WSGI server
- Configure reverse proxy (nginx)
- Set up SSL/HTTPS for security

## Integration

### ROS2 Integration

The Dash version maintains full compatibility with ROS2 systems:

- MQTT/Redis messaging
- Standard pose and scan topics
- nav2_simple_commander support

### External Systems

- REST API endpoints can be added
- WebSocket connections for real-time data
- Database integration for logging

## License

Same license as the original TurtleBot simulator project.
