# TurtleBot LiDAR Occlusion Simulator

A comprehensive web-based TurtleBot simulation dashboard with LiDAR occlusion simulation, trustworthiness monitoring, and adaptive navigation capabilities.

## 🌟 Features

### 🤖 Robot Simulation
- **Real-time Position Tracking**: Live robot position, heading, and trajectory
- **LiDAR Simulation**: Normal and occluded sensor data with 360-degree visualization
- **Dual Navigation Modes**: Standard navigation and spin configuration
- **Obstacle Avoidance**: A* pathfinding algorithm with dynamic obstacle generation
- **Random Walk**: Autonomous random navigation mode

### 🛡️ Trustworthiness System
- **Real-time Trustworthiness Monitoring**: Subscribe to `maple` topic for trust status
- **Adaptive Failure Handling**: Three configurable failure responses:
  - **Apply Adaptation**: Execute spin configuration when trustworthiness fails
  - **Stop Robot**: Halt all navigation when trust is compromised
  - **Continue Standard**: Maintain normal navigation despite trust failure
- **Manual Trust Toggle**: Test button for simulating trust state changes

### 🌐 Web-Based Dashboard
- **Modern UI**: Bootstrap-based responsive design with real-time updates
- **Interactive Map**: Click-to-navigate functionality with live trajectory display
- **Polar LiDAR Plot**: Real-time 360-degree sensor data visualization
- **Status Indicators**: Live system status with color-coded badges
- **Control Panel**: Comprehensive control interface with dropdown menus

### 📡 Communication
- **Redis Integration**: Pub/Sub messaging for external system integration
- **MQTT Compatibility**: Support for external navigation commands
- **Real-time Logging**: Redis-based logging with structured output

## 🚀 Quick Start

### Prerequisites
- Python 3.8+
- Redis server (local or Docker)
- Web browser

### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/saharnn96/Turtlebot_LidarOcclusion_Simulator.git
   cd Turtlebot_LidarOcclusion_Simulator
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run with Docker** (Recommended):
   ```bash
   docker-compose up
   ```

4. **Or run locally**:
   ```bash
   python dash_turtlebotsim.py
   ```

5. **Access Dashboard**:
   - Docker: http://localhost:8051
   - Local: http://localhost:8050

## 🎮 Usage Guide

### Dashboard Controls

#### Status Indicators
- **LiDAR Status**: Shows NORMAL/OCCLUDED state
- **Navigation Mode**: Shows STANDARD/SPIN_CONFIG state
- **Trustworthiness**: Shows TRUSTED/UNTRUSTED state

#### Control Buttons
- **Toggle LiDAR Occlusion**: Simulate sensor occlusion (first 300 readings → ∞)
- **Toggle Navigation Mode**: Switch between standard and spin configuration
- **Stop Navigation**: Emergency stop for all navigation activities
- **Toggle Trustworthiness**: Manual trust state testing

#### Navigation Controls
- **Manual Navigation**: Enter X,Y coordinates for direct navigation
- **Random Walk**: Enable continuous random navigation
- **Map Click**: Click anywhere on the map to navigate there

#### Trustworthiness Failure Action
Configure system response when trustworthiness fails:
- **Apply Adaptation**: Use spin configuration parameters
- **Stop Robot**: Halt all movement
- **Continue Standard**: Ignore trust failure and continue

### External Communication

#### Redis Topics

The simulator subscribes to two main topics:

##### 1. Spin Configuration: `/spin_config`
Controls robot navigation behavior with adaptive trustworthiness checking.

**Message Format**:
```json
{
  "commands": [
    {
      "duration": 1.0,
      "omega": 45
    }
  ]
}
```

**Parameters**:
- `duration`: Navigation duration (0.0 = standard navigation, >0.0 = spin config)
- `omega`: Rotation angle in degrees for spin configuration

##### 2. Trustworthiness Status: `maple`
Receives trust status from external trustworthiness checker.

**Message Format**:
```json
{
  "Bool": true
}
```

**Parameters**:
- `Bool`: Trust status (true = trusted, false = untrusted)

## 🔧 Redis Commands

### Testing Trustworthiness System

#### Basic Commands

**Set trustworthiness to FALSE (untrusted)**:
```bash
redis-cli PUBLISH maple '{"Bool":false}'
```

**Set trustworthiness to TRUE (trusted)**:
```bash
redis-cli PUBLISH maple '{"Bool":true}'
```

**Send spin configuration**:
```bash
redis-cli PUBLISH /spin_config '{"commands":[{"duration":1.0,"omega":45}]}'
```

**Reset to standard navigation**:
```bash
redis-cli PUBLISH /spin_config '{"commands":[{"duration":0.0,"omega":0}]}'
```

#### Test Scenarios

**1. Test Adaptation on Trust Failure**:
```bash
# First set trustworthiness to false
redis-cli PUBLISH maple '{"Bool":false}'

# Then send spin config - this should trigger the failure action
redis-cli PUBLISH /spin_config '{"commands":[{"duration":2.0,"omega":90}]}'
```

**2. Test Normal Operation When Trusted**:
```bash
# Set trustworthiness to true
redis-cli PUBLISH maple '{"Bool":true}'

# Send spin config - this should work normally
redis-cli PUBLISH /spin_config '{"commands":[{"duration":1.5,"omega":60}]}'
```

**3. Test Different Failure Actions**:
```bash
# Set failure action in UI dropdown, then:
redis-cli PUBLISH maple '{"Bool":false}'
redis-cli PUBLISH /spin_config '{"commands":[{"duration":1.0,"omega":30}]}'
```

#### Docker Commands

If using Docker setup:
```bash
# Connect to redis container and run commands
docker exec -it turtlebot_redis redis-cli PUBLISH maple '{"Bool":false}'
docker exec -it turtlebot_redis redis-cli PUBLISH /spin_config '{"commands":[{"duration":1.0,"omega":45}]}'
```

#### Monitoring Messages

**Subscribe to all topics**:
```bash
redis-cli PSUBSCRIBE "*"
```

**Monitor specific topics**:
```bash
redis-cli SUBSCRIBE maple /spin_config
```

## 🔄 System Behavior

### Trustworthiness Logic Flow

1. **Receive Spin Config**: System receives navigation command on `/spin_config`
2. **Check Duration**: 
   - If `duration == 0.0` → Switch to standard navigation
   - If `duration > 0.0` → Check trustworthiness status
3. **Trustworthiness Check**:
   - If **TRUSTED** → Execute spin configuration normally
   - If **UNTRUSTED** → Apply configured failure action:
     - **Apply Adaptation**: Execute spin config as adaptation strategy
     - **Stop Robot**: Halt all navigation immediately
     - **Continue Standard**: Ignore spin config, continue standard navigation

### Navigation Modes

#### Standard Navigation
- Direct point-to-point movement
- Single pose publication per waypoint
- Faster execution

#### Spin Configuration
- Enhanced movement with rotation
- Three-phase execution per waypoint:
  1. Move to position + publish pose/scan
  2. Rotate by `omega` angle + publish pose/scan  
  3. Rotate back by `omega` angle + publish pose/scan
- Slower but more thorough sensing

## 📊 Monitoring & Debugging

### Log Monitoring

**View Redis logs**:
```bash
redis-cli LRANGE Simulator:logs 0 -1
```

**Monitor real-time logs**:
```bash
redis-cli --latency-history PUBLISH test "monitor"
```

### Dashboard Monitoring

The dashboard provides real-time status in the Robot Status panel:
- Current position and heading
- Navigation state (Active/Idle)
- Random walk status
- Current failure action setting
- Obstacle count

## 🐳 Docker Setup

### Services

The `docker-compose.yml` includes:

1. **Redis Service**: Message broker and logging
   - Port: 6379
   - Container: `turtlebot_redis`

2. **TurtleBot Dash Simulator**: Main application
   - Port: 8051 (external) → 8051 (internal)
   - Container: `turtlebot_dash_simulator`

### Environment Variables

- `DASH_HOST`: Server host (default: 0.0.0.0)
- `DASH_PORT`: Server port (default: 8051 in Docker, 8050 locally)
- `REDIS_HOST`: Redis server host (default: redis in Docker, localhost locally)
- `REDIS_PORT`: Redis server port (default: 6379)
- `DASH_DEBUG`: Debug mode (default: False)

## 🔧 Configuration

### Customizing Behavior

Edit `dash_turtlebotsim.py` to modify:

**Map Settings**:
```python
self.map_size = 10  # Map boundaries (-10 to +10)
num_obstacles = 5   # Number of random obstacles
```

**Navigation Settings**:
```python
steps = 20  # A* pathfinding resolution
time.sleep(0.5)  # Navigation speed
```

**LiDAR Settings**:
```python
self.lidar_data = [random.uniform(5, 10) for _ in range(360)]  # Normal range
self.lidar_data[0:300] = [float('inf') for _ in range(300)]   # Occlusion pattern
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Troubleshooting

### Common Issues

**Dashboard not loading**:
- Check if port 8050/8051 is available
- Verify Python dependencies are installed
- Check Redis connection

**Redis connection failed**:
- Ensure Redis server is running
- Check Redis host/port configuration
- For Docker: ensure containers are on same network

**Trustworthiness not updating**:
- Verify message format: `{"Bool": true/false}`
- Check topic name: `maple`
- Monitor Redis with `redis-cli MONITOR`

**Navigation not working**:
- Check if robot is in navigation state
- Verify trajectory generation
- Check for emergency stop state

### Debug Commands

**Test Redis connection**:
```bash
redis-cli ping
```

**Check container status**:
```bash
docker-compose ps
```

**View container logs**:
```bash
docker-compose logs turtlebot_dash
```

---

For more detailed technical information, see [DASH_README.md](DASH_README.md).
