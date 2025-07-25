import time
import threading
import json
import os
import socket
import logging
from datetime import datetime
import numpy as np
import paho.mqtt.client as mqtt
import random
import plotly.graph_objs as go
import plotly.express as px
from plotly.subplots import make_subplots
import dash
from dash import dcc, html, Input, Output, State, callback_context
import dash_bootstrap_components as dbc
from rpio.clientLibraries.rpclpy.CommunicationManager import CommunicationManager

# MQTT setup
MQTT_BROKER = "localhost"
POSE_TOPIC = "/pose"
SCAN_TOPIC = "/Scan"
SPIN_CONFIG_TOPIC = "/spin_config"

# Redis logging class
class RedisLogger:
    def __init__(self, client, container_name):
        self.client = client
        self.container_name = container_name
        self.log_key = f"{container_name}:logs"
        
    def log(self, level, message):
        """Log a message to Redis"""
        try:
            timestamp = datetime.now().isoformat()
            log_entry = {
                "timestamp": timestamp,
                "level": level,
                "message": message,
                "container": self.container_name
            }
            
            if self.client:
                # Store as a list in Redis (LPUSH adds to the beginning)
                self.client.client.lpush(self.log_key, json.dumps(log_entry))
                # Keep only last 1000 log entries
                self.client.client.ltrim(self.log_key, 0, 999)
                
        except Exception as e:
            # Fallback to console if Redis logging fails
            print(f"[{timestamp}] [{level}] {message}")
            print(f"Redis logging error: {e}")
    
    def info(self, message):
        self.log("INFO", message)
        
    def warning(self, message):
        self.log("WARNING", message)
        
    def error(self, message):
        self.log("ERROR", message)
        
    def debug(self, message):
        self.log("DEBUG", message)

# Global logger instance
logger = None

# TurtleBotSim class
class TurtleBotSim:
    def __init__(self):
        self.position = [0, 0]
        self.angle = 1.0
        self.heading = 0
        self.lidar_occluded = False
        self.standard_navigation = True
        self.map_size = 10
        self.obstacles = self.generate_obstacles()
        self.trajectory = []
        self.lidar_data = [random.uniform(5, 10) for _ in range(360)]
        self.navigation_active = False
        
    def generate_obstacles(self, num_obstacles=5):
        obstacles = []
        for _ in range(num_obstacles):
            obstacles.append((random.uniform(-self.map_size, self.map_size), 
                            random.uniform(-self.map_size, self.map_size)))
        return obstacles
    
    def publish_pose(self):
        if hasattr(self, 'client') and self.client:
            client.publish(POSE_TOPIC, json.dumps({
                "x": self.position[0], 
                "y": self.position[1], 
                "angle": self.angle
            }))

    def publish_scan(self):
        if self.lidar_occluded:
            self.lidar_data[0:300] = [float('inf') for _ in range(300)]
        else:
            self.lidar_data = [random.uniform(5, 10) for _ in range(360)]
        
        lidar_data = {
            'angle_min': -3.124,
            'angle_max': 3.1415927,
            'angle_increment': 0.0174533,
            'scan_time': 0.2,
            'range_min': 0.1,
            'range_max': 12.0,
            'ranges': self.lidar_data
        }
        
        if hasattr(self, 'client') and self.client:
            client.publish(SCAN_TOPIC, json.dumps(lidar_data))

    def navigate_to(self, trajectory):
        self.trajectory = trajectory
        self.navigation_active = True
        
        if logger:
            logger.info(f"Navigation started with {len(trajectory)} waypoints")
        
        for i, point in enumerate(trajectory):
            if not self.navigation_active:  # Allow stopping navigation
                if logger:
                    logger.info("Navigation stopped early")
                break
                
            self.heading = np.atan2(point[1]-self.position[1], point[0]-self.position[0])
            self.position = point
            
            if logger and i % 5 == 0:  # Log every 5th waypoint to avoid spam
                logger.debug(f"Waypoint {i+1}/{len(trajectory)}: ({point[0]:.2f}, {point[1]:.2f})")
            
            if self.standard_navigation:
                self.publish_pose()
                self.publish_scan()
                time.sleep(0.5)
            else:
                self.publish_pose()
                self.publish_scan()
                time.sleep(0.5)
                self.heading = self.heading + self.angle
                self.publish_pose()
                self.publish_scan()
                time.sleep(0.5)
                self.heading = self.heading - self.angle
                self.publish_pose()
                self.publish_scan()
                time.sleep(0.5)
        
        self.navigation_active = False
        if logger:
            logger.info(f"Navigation completed. Final position: ({self.position[0]:.2f}, {self.position[1]:.2f})")

    def spin(self, duration, angle):
        self.angle += angle
        self.publish_pose()
        time.sleep(duration)

    def stop_navigation(self):
        self.navigation_active = False

# A* Pathfinding (Simple Implementation)
def astar(start, end, obstacles):
    path = []
    steps = 20
    x_vals = np.linspace(start[0], end[0], steps)
    y_vals = np.linspace(start[1], end[1], steps)
    for i in range(steps):
        path.append([x_vals[i], y_vals[i]])
    return path

# Initialize simulator
sim = TurtleBotSim()

# Initialize Dash app with Bootstrap theme
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.title = "TurtleBot4 Simulation Dashboard"

# App layout
app.layout = dbc.Container([
    dbc.Row([
        dbc.Col([
            html.H1("TurtleBot4 Simulation Dashboard", 
                   className="text-center mb-4 text-primary"),
        ], width=12)
    ]),
    
    dbc.Row([
        # Control Panel
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("Control Panel"),
                dbc.CardBody([
                    # Status indicators
                    html.Div([
                        dbc.Badge("LiDAR Status", color="secondary", className="me-2"),
                        dbc.Badge(id="lidar-status", color="success", className="mb-2"),
                    ]),
                    html.Div([
                        dbc.Badge("Navigation Mode", color="secondary", className="me-2"),
                        dbc.Badge(id="nav-mode-status", color="info", className="mb-3"),
                    ]),
                    
                    # Control buttons
                    dbc.ButtonGroup([
                        dbc.Button("Toggle LiDAR Occlusion", 
                                 id="toggle-lidar-btn", 
                                 color="warning", 
                                 className="mb-2"),
                        dbc.Button("Toggle Navigation Mode", 
                                 id="toggle-nav-btn", 
                                 color="info", 
                                 className="mb-2"),
                        dbc.Button("Stop Navigation", 
                                 id="stop-nav-btn", 
                                 color="danger", 
                                 className="mb-2"),
                    ], vertical=True, className="d-grid gap-2"),
                    
                    html.Hr(),
                    
                    # Manual position input
                    html.H6("Manual Navigation"),
                    dbc.Row([
                        dbc.Col([
                            dbc.Label("Target X:"),
                            dbc.Input(id="target-x", type="number", value=0, step=0.1),
                        ], width=6),
                        dbc.Col([
                            dbc.Label("Target Y:"),
                            dbc.Input(id="target-y", type="number", value=0, step=0.1),
                        ], width=6),
                    ]),
                    dbc.Button("Navigate to Target", 
                             id="navigate-btn", 
                             color="success", 
                             className="mt-2 w-100"),
                    
                    html.Hr(),
                    
                    # Robot status
                    html.H6("Robot Status"),
                    html.Div(id="robot-status", className="small"),
                ])
            ])
        ], width=3),
        
        # Visualization Panel
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("Simulation Visualization"),
                dbc.CardBody([
                    dcc.Graph(id="map-plot", style={"height": "500px"}),
                ])
            ])
        ], width=6),
        
        # LiDAR Panel
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("LiDAR Data"),
                dbc.CardBody([
                    dcc.Graph(id="lidar-plot", style={"height": "500px"}),
                ])
            ])
        ], width=3),
    ]),
    
    # Auto-refresh interval
    dcc.Interval(
        id='interval-component',
        interval=1000,  # Update every second
        n_intervals=0
    ),
    
    # Store components for state management
    dcc.Store(id='click-data'),
    dcc.Store(id='navigation-thread'),
    
], fluid=True)

# Callback for updating plots
@app.callback(
    [Output('map-plot', 'figure'),
     Output('lidar-plot', 'figure'),
     Output('lidar-status', 'children'),
     Output('lidar-status', 'color'),
     Output('nav-mode-status', 'children'),
     Output('nav-mode-status', 'color'),
     Output('robot-status', 'children')],
    [Input('interval-component', 'n_intervals')]
)
def update_plots(n):
    # Create map plot
    map_fig = go.Figure()
    
    # Add robot position
    map_fig.add_trace(go.Scatter(
        x=[sim.position[0]], 
        y=[sim.position[1]],
        mode='markers+text',
        marker=dict(size=15, color='blue', symbol='circle'),
        text=['TurtleBot4'],
        textposition="top center",
        name='Robot'
    ))
    
    # Add heading arrow
    heading_x = sim.position[0] + 1 * np.cos(sim.heading)
    heading_y = sim.position[1] + 1 * np.sin(sim.heading)
    map_fig.add_trace(go.Scatter(
        x=[sim.position[0], heading_x], 
        y=[sim.position[1], heading_y],
        mode='lines',
        line=dict(color='blue', width=3),
        name='Heading'
    ))
    
    # Add trajectory
    if sim.trajectory:
        traj_x = [point[0] for point in sim.trajectory]
        traj_y = [point[1] for point in sim.trajectory]
        map_fig.add_trace(go.Scatter(
            x=traj_x, 
            y=traj_y,
            mode='lines+markers',
            line=dict(color='green', dash='dash'),
            marker=dict(size=4),
            name='Trajectory'
        ))
    
    # Add obstacles
    if sim.obstacles:
        obs_x = [obs[0] for obs in sim.obstacles]
        obs_y = [obs[1] for obs in sim.obstacles]
        map_fig.add_trace(go.Scatter(
            x=obs_x, 
            y=obs_y,
            mode='markers',
            marker=dict(size=12, color='red', symbol='x'),
            name='Obstacles'
        ))
    
    map_fig.update_layout(
        title="TurtleBot4 Map",
        xaxis_title="X Position",
        yaxis_title="Y Position",
        xaxis=dict(range=[-sim.map_size, sim.map_size]),
        yaxis=dict(range=[-sim.map_size, sim.map_size]),
        showlegend=True,
        height=450,
        hovermode='closest'
    )
    
    # Create LiDAR plot (polar)
    angles = np.linspace(0, 2 * np.pi, len(sim.lidar_data))
    angles_deg = np.degrees(angles)
    
    lidar_fig = go.Figure()
    lidar_fig.add_trace(go.Scatterpolar(
        r=sim.lidar_data,
        theta=angles_deg,
        mode='lines',
        line=dict(color='red'),
        name='LiDAR Data'
    ))
    
    lidar_fig.update_layout(
        title="LiDAR Data",
        polar=dict(
            radialaxis=dict(range=[0, 10], showticklabels=True),
            angularaxis=dict(direction="counterclockwise", period=360)
        ),
        height=450
    )
    
    # Status updates
    lidar_status = "OCCLUDED" if sim.lidar_occluded else "NORMAL"
    lidar_color = "danger" if sim.lidar_occluded else "success"
    
    nav_mode = "STANDARD" if sim.standard_navigation else "SPIN_CONFIG"
    nav_color = "primary" if sim.standard_navigation else "warning"
    
    robot_status = html.Div([
        html.P(f"Position: ({sim.position[0]:.2f}, {sim.position[1]:.2f})"),
        html.P(f"Heading: {np.degrees(sim.heading):.1f}°"),
        html.P(f"Angle: {sim.angle:.1f}"),
        html.P(f"Navigation: {'Active' if sim.navigation_active else 'Idle'}"),
        html.P(f"Obstacles: {len(sim.obstacles)}")
    ])
    
    return map_fig, lidar_fig, lidar_status, lidar_color, nav_mode, nav_color, robot_status

# Callback for LiDAR toggle
@app.callback(
    Output('toggle-lidar-btn', 'children'),
    [Input('toggle-lidar-btn', 'n_clicks')]
)
def toggle_lidar(n_clicks):
    if n_clicks:
        sim.lidar_occluded = not sim.lidar_occluded
        status = "ON" if sim.lidar_occluded else "OFF"
        if logger:
            logger.info(f"LiDAR occlusion toggled: {status}")
    return f"Toggle LiDAR Occlusion ({'ON' if sim.lidar_occluded else 'OFF'})"

# Callback for navigation mode toggle
@app.callback(
    Output('toggle-nav-btn', 'children'),
    [Input('toggle-nav-btn', 'n_clicks')]
)
def toggle_nav_mode(n_clicks):
    if n_clicks:
        sim.standard_navigation = not sim.standard_navigation
        mode = "STANDARD" if sim.standard_navigation else "SPIN_CONFIG"
        if logger:
            logger.info(f"Navigation mode changed to: {mode}")
    mode = "STANDARD" if sim.standard_navigation else "SPIN_CONFIG"
    return f"Mode: {mode}"

# Callback for stopping navigation
@app.callback(
    Output('stop-nav-btn', 'children'),
    [Input('stop-nav-btn', 'n_clicks')]
)
def stop_navigation(n_clicks):
    if n_clicks:
        sim.stop_navigation()
        if logger:
            logger.info("Navigation stopped by user")
    return "Stop Navigation"

# Callback for manual navigation
@app.callback(
    Output('navigate-btn', 'children'),
    [Input('navigate-btn', 'n_clicks')],
    [State('target-x', 'value'),
     State('target-y', 'value')]
)
def manual_navigate(n_clicks, target_x, target_y):
    if n_clicks and target_x is not None and target_y is not None:
        target_coords = [target_x, target_y]
        trajectory = astar(sim.position, target_coords, sim.obstacles)
        
        if logger:
            logger.info(f"Manual navigation started to ({target_x}, {target_y})")
        
        # Start navigation in a separate thread
        nav_thread = threading.Thread(target=sim.navigate_to, args=(trajectory,))
        nav_thread.daemon = True
        nav_thread.start()
        
        return f"Navigating to ({target_x}, {target_y})"
    return "Navigate to Target"

# Callback for map clicks
@app.callback(
    Output('click-data', 'data'),
    [Input('map-plot', 'clickData')]
)
def handle_map_click(clickData):
    if clickData and 'points' in clickData and len(clickData['points']) > 0:
        point = clickData['points'][0]
        target_coords = [point['x'], point['y']]
        trajectory = astar(sim.position, target_coords, sim.obstacles)
        
        if logger:
            logger.info(f"Map click navigation to ({point['x']:.2f}, {point['y']:.2f})")
        
        # Start navigation in a separate thread
        nav_thread = threading.Thread(target=sim.navigate_to, args=(trajectory,))
        nav_thread.daemon = True
        nav_thread.start()
        
        return {'x': point['x'], 'y': point['y']}
    return {}

# MQTT message handler
def on_message(message):
    try:
        payload = json.loads(message)
        if payload.get("commands"):
            plan = payload.get("commands")[0]
            duration = plan.get("duration")
            if duration == 0.0:
                sim.standard_navigation = True
                if logger:
                    logger.info("External command: Standard navigation enabled")
            else:
                sim.standard_navigation = False
                sim.angle = plan.get("omega", 90)
                if logger:
                    logger.info(f"External command: Spin config enabled (omega={sim.angle})")
    except json.JSONDecodeError:
        sim.standard_navigation = True
        if logger:
            logger.warning("Invalid JSON in spin config message")
        print("Invalid JSON in spin config")

# Initialize MQTT client
def initialize_client():
    global client, logger
    try:
        # Get Redis configuration from environment variables
        redis_host = os.getenv('REDIS_HOST', 'localhost')
        redis_port = int(os.getenv('REDIS_PORT', '6379'))
        
        # Get container name (defaults to hostname if not in container)
        container_name = os.getenv('HOSTNAME', socket.gethostname())
        
        client = CommunicationManager({
            "protocol": "redis", 
            "host": redis_host, 
            "port": redis_port
        })
        client.subscribe(SPIN_CONFIG_TOPIC, callback=on_message)
        client.start()
        sim.client = client
        
        # Initialize Redis logger
        logger = RedisLogger(client, container_name)
        logger.info(f"Connected to Redis at {redis_host}:{redis_port}")
        logger.info(f"Container: {container_name}")
        logger.info("TurtleBot Dash Simulator initialized")
        
        print(f"✅ Connected to Redis at {redis_host}:{redis_port}")
        print(f"📝 Logging to Redis key: {container_name}:logs")
        
    except Exception as e:
        print(f"⚠️  Failed to initialize communication client: {e}")
        print("📝 Simulator will run without external communication")
        client = None
        logger = None

if __name__ == "__main__":
    # Get configuration from environment variables
    dash_host = os.getenv('DASH_HOST', '0.0.0.0')
    dash_port = int(os.getenv('DASH_PORT', '8050'))
    dash_debug = os.getenv('DASH_DEBUG', 'False').lower() == 'true'
    
    # Initialize MQTT client and logger
    initialize_client()
    
    if logger:
        logger.info("Starting TurtleBot Dash Simulator")
        logger.info(f"Dashboard configuration: {dash_host}:{dash_port}")
        logger.info(f"Debug mode: {dash_debug}")
    
    print(f"🚀 Starting TurtleBot Dash Simulator")
    print(f"🌐 Dashboard will be available at: http://{dash_host}:{dash_port}")
    print(f"🐛 Debug mode: {dash_debug}")
    
    try:
        # Run the Dash app
        app.run(debug=dash_debug, host=dash_host, port=dash_port)
    except Exception as e:
        if logger:
            logger.error(f"Failed to start Dash server: {e}")
        print(f"❌ Failed to start server: {e}")
    finally:
        if logger:
            logger.info("TurtleBot Dash Simulator shutting down")
