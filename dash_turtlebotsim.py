import time
import threading
import json
import os
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
import logging
import redis



# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# Redis log handler
class RedisLogHandler(logging.Handler):
    def __init__(self, redis_host='localhost', redis_port=6379, key='Simulator:logs'):
        super().__init__()
        self.redis = redis.StrictRedis(host=os.getenv('REDIS_HOST', 'localhost'), port=int(os.getenv('REDIS_PORT', '6379')), decode_responses=True)
        self.key = key

    def emit(self, record):
        try:
            msg = self.format(record)
            self.redis.lpush(self.key, msg)
        except Exception:
            self.handleError(record)

# Attach Redis handler to root logger
redis_handler = RedisLogHandler()
redis_handler.setLevel(logging.INFO)
redis_handler.setFormatter(logging.Formatter('[%(asctime)s] [%(levelname)s] %(message)s', '%Y-%m-%d %H:%M:%S'))
logging.getLogger().addHandler(redis_handler)

MQTT_BROKER = "localhost"
POSE_TOPIC = "/pose"
SCAN_TOPIC = "/Scan"
SPIN_CONFIG_TOPIC = "/spin_config"
TRUSTWORTHINESS_TOPIC = "maple"


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
        self.random_walk_active = False  # NEW: flag for random walk mode
        self.trustworthiness_status = True  # NEW: trustworthiness status from maple topic
        self.failure_action = "stop_robot"  # NEW: action to take on trustworthiness failure
        # NEW: Selective LiDAR occlusion
        self.lidar_occlusion_sectors = {
            'front': False,      # 315-360 and 0-45 degrees
            'front_left': False, # 45-90 degrees
            'left': False,       # 90-135 degrees
            'back_left': False,  # 135-180 degrees
            'back': False,       # 180-225 degrees
            'back_right': False, # 225-270 degrees
            'right': False,      # 270-315 degrees
        }
        # Custom occlusion ranges (start_angle, end_angle)
        self.custom_occlusion_ranges = []
        # Random occlusion feature
        self.random_occlusion_active = False
        self.random_occlusion_thread = None
        logging.info("TurtleBotSim initialized with position (0, 0) and angle 1.0")
        
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
            logging.debug(f"Published pose: x={self.position[0]}, y={self.position[1]}, angle={self.angle}")

    def publish_scan(self):
        # Start with fresh random data
        self.lidar_data = [random.uniform(5, 10) for _ in range(360)]
        
        # Apply global occlusion (backward compatibility)
        if self.lidar_occluded:
            self.lidar_data[0:300] = [float('inf') for _ in range(300)]
            logging.info("Global lidar occlusion active: first 300 readings set to inf")
        
        # Apply selective sector occlusion
        for sector, is_occluded in self.lidar_occlusion_sectors.items():
            if is_occluded:
                start_angle, end_angle = self.get_sector_range(sector)
                self.occlude_angle_range(start_angle, end_angle)
                logging.debug(f"Sector '{sector}' occluded: {start_angle}-{end_angle} degrees")
        
        # Apply custom occlusion ranges
        for start_angle, end_angle in self.custom_occlusion_ranges:
            self.occlude_angle_range(start_angle, end_angle)
            logging.debug(f"Custom occlusion applied: {start_angle}-{end_angle} degrees")
        
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

    def get_sector_range(self, sector):
        """Get angle range for predefined sectors."""
        sector_ranges = {
            'front': (315, 45),      # Wraps around 0
            'front_left': (45, 90),
            'left': (90, 135),
            'back_left': (135, 180),
            'back': (180, 225),
            'back_right': (225, 270),
            'right': (270, 315),
        }
        return sector_ranges.get(sector, (0, 360))

    def occlude_angle_range(self, start_angle, end_angle):
        """Occlude LiDAR readings in the specified angle range."""
        # Convert angles to array indices (0-359)
        start_idx = int(start_angle)
        end_idx = int(end_angle)
        
        if start_idx <= end_idx:
            # Normal range
            for i in range(start_idx, min(end_idx + 1, 360)):
                self.lidar_data[i] = float('inf')
        else:
            # Wrap-around range (e.g., 315-45 degrees for front)
            for i in range(start_idx, 360):
                self.lidar_data[i] = float('inf')
            for i in range(0, end_idx + 1):
                self.lidar_data[i] = float('inf')

    def toggle_sector_occlusion(self, sector):
        """Toggle occlusion for a specific sector."""
        if sector in self.lidar_occlusion_sectors:
            self.lidar_occlusion_sectors[sector] = not self.lidar_occlusion_sectors[sector]
            status = "ON" if self.lidar_occlusion_sectors[sector] else "OFF"
            logging.info(f"Sector '{sector}' occlusion toggled: {status}")

    def add_custom_occlusion(self, start_angle, end_angle):
        """Add a custom occlusion range."""
        self.custom_occlusion_ranges.append((start_angle, end_angle))
        logging.info(f"Custom occlusion added: {start_angle}-{end_angle} degrees")

    def clear_custom_occlusions(self):
        """Clear all custom occlusion ranges."""
        self.custom_occlusion_ranges.clear()
        logging.info("All custom occlusions cleared")

    def clear_all_occlusions(self):
        """Clear all occlusions (sectors and custom)."""
        for sector in self.lidar_occlusion_sectors:
            self.lidar_occlusion_sectors[sector] = False
        self.custom_occlusion_ranges.clear()
        self.lidar_occluded = False
        # Stop random occlusion if active
        if self.random_occlusion_active:
            self.stop_random_occlusion()
        logging.info("All LiDAR occlusions cleared")

    def get_occlusion_status(self):
        """Get current occlusion status as a string."""
        active_sectors = [sector for sector, active in self.lidar_occlusion_sectors.items() if active]
        status_parts = []
        
        if self.lidar_occluded:
            status_parts.append("Global: ON")
        
        if active_sectors:
            status_parts.append(f"Sectors: {', '.join(active_sectors)}")
        
        if self.custom_occlusion_ranges:
            custom_ranges = [f"{start}-{end}°" for start, end in self.custom_occlusion_ranges]
            status_parts.append(f"Custom: {', '.join(custom_ranges)}")
        
        if self.random_occlusion_active:
            status_parts.append("Random: ON")
        
        return " | ".join(status_parts) if status_parts else "No occlusions"

    def start_random_occlusion(self):
        """Start random occlusion that changes every 3 seconds."""
        if self.random_occlusion_active:
            return  # Already running
        
        self.random_occlusion_active = True
        self.random_occlusion_thread = threading.Thread(target=self._random_occlusion_loop)
        self.random_occlusion_thread.daemon = True
        self.random_occlusion_thread.start()
        logging.info("Random LiDAR occlusion started - changes every 3 seconds")

    def stop_random_occlusion(self):
        """Stop random occlusion."""
        self.random_occlusion_active = False
        if self.random_occlusion_thread:
            self.random_occlusion_thread.join(timeout=1)
        logging.info("Random LiDAR occlusion stopped")

    def _random_occlusion_loop(self):
        """Internal method that runs the random occlusion loop."""
        import time
        import random
        
        while self.random_occlusion_active:
            try:
                # Clear previous random occlusions
                self.clear_custom_occlusions()
                
                # Generate 1-3 random occlusion sectors
                num_occlusions = random.randint(1, 3)
                
                for _ in range(num_occlusions):
                    # Generate random start angle (0-359)
                    start_angle = random.randint(0, 359)
                    # Generate random arc size (15-90 degrees)
                    arc_size = random.randint(15, 90)
                    end_angle = (start_angle + arc_size) % 360
                    
                    # Add the random occlusion
                    self.add_custom_occlusion(start_angle, end_angle)
                
                logging.debug(f"Applied {num_occlusions} random occlusions")
                
                # Wait for 3 seconds before next change
                time.sleep(3.0)
                
            except Exception as e:
                logging.error(f"Error in random occlusion loop: {e}")
                break
        
        # Clean up when stopping
        self.clear_custom_occlusions()
        logging.info("Random occlusion loop ended")

    def navigate_to(self, trajectory):
        self.trajectory = trajectory
        self.navigation_active = True
        logging.info(f"Navigation started with {len(trajectory)} waypoints.")
        for i, point in enumerate(trajectory):
            if not self.navigation_active:  # Allow stopping navigation
                break
            # Use numpy.arctan2 (correct function name) to compute heading
            self.heading = np.arctan2(point[1]-self.position[1], point[0]-self.position[0])
            self.position = point
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
        logging.info(f"Navigation ended. Final position: {self.position}")

    def spin(self, duration, angle):
        self.angle += angle
        self.publish_pose()
        time.sleep(duration)
        logging.info(f"Spin executed: duration={duration}, angle={angle}")

    def random_walk_loop(self):
        """Continuously pick random targets and navigate until stopped."""
        if self.random_walk_active:
            # already running
            return
        self.random_walk_active = True
        self.navigation_active = True
        logging.info("Random walk started.")
        try:
            while self.random_walk_active:
                target = [random.uniform(-self.map_size, self.map_size),
                          random.uniform(-self.map_size, self.map_size)]
                trajectory = astar(self.position, target, self.obstacles)
                self.trajectory = trajectory
                logging.info(f"Random walk target: ({target[0]:.2f}, {target[1]:.2f}) with {len(trajectory)} waypoints")
                for point in trajectory:
                    if not self.random_walk_active:
                        break
                    self.heading = np.arctan2(point[1]-self.position[1], point[0]-self.position[0])
                    self.position = point
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
        finally:
            self.navigation_active = False
            self.random_walk_active = False
            logging.info("Random walk stopped.")

    def stop_navigation(self):
        self.navigation_active = False
        self.random_walk_active = False  # ensure random walk loop exits
        # Optional: stop random occlusion when navigation stops
        # self.stop_random_occlusion()

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

# Clear any potential cache issues
app.config.suppress_callback_exceptions = True

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
                    html.Div([
                        dbc.Badge("Trustworthiness", color="secondary", className="me-2"),
                        dbc.Badge(id="trustworthiness-status", color="success", className="mb-3"),
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
                        dbc.Button("Toggle Trustworthiness", 
                                 id="toggle-trust-btn", 
                                 color="secondary", 
                                 className="mb-2"),
                    ], vertical=True, className="d-grid gap-2"),
                    
                    html.Hr(),
                    
                    # Trustworthiness failure action
                    html.H6("Trustworthiness Failure Action"),
                    dbc.Label("If trustworthiness fails:"),
                    dcc.Dropdown(
                        id="failure-action-dropdown",
                        options=[
                            {"label": "Apply Adaptation", "value": "apply_adaptation"},
                            {"label": "Stop Robot", "value": "stop_robot"},
                            {"label": "Continue Standard Navigation", "value": "continue_standard"}
                        ],
                        value="apply_adaptation",
                        className="mb-3"
                    ),
                    
                    html.Hr(),
                    
                    # Selective LiDAR Occlusion Controls
                    html.H6("Selective LiDAR Occlusion"),
                    html.Div([
                        html.P("Sector Controls:", className="mb-2 small text-muted"),
                        dbc.Row([
                            dbc.Col([
                                dbc.Checklist(
                                    id="lidar-sector-checklist",
                                    options=[
                                        {"label": "Front", "value": "front"},
                                        {"label": "Front-Left", "value": "front_left"},
                                        {"label": "Left", "value": "left"},
                                        {"label": "Back-Left", "value": "back_left"},
                                    ],
                                    value=[],
                                    inline=False
                                ),
                            ], width=6),
                            dbc.Col([
                                dbc.Checklist(
                                    id="lidar-sector-checklist-2",
                                    options=[
                                        {"label": "Back", "value": "back"},
                                        {"label": "Back-Right", "value": "back_right"},
                                        {"label": "Right", "value": "right"},
                                    ],
                                    value=[],
                                    inline=False
                                ),
                            ], width=6),
                        ]),
                        
                        html.Hr(className="my-2"),
                        
                        # Custom angle range occlusion
                        html.P("Custom Angle Range:", className="mb-2 small text-muted"),
                        dbc.Row([
                            dbc.Col([
                                dbc.Label("Start Angle (°):", className="small"),
                                dbc.Input(
                                    id="custom-start-angle",
                                    type="number",
                                    min=0,
                                    max=359,
                                    value=0,
                                    size="sm"
                                ),
                            ], width=6),
                            dbc.Col([
                                dbc.Label("End Angle (°):", className="small"),
                                dbc.Input(
                                    id="custom-end-angle",
                                    type="number",
                                    min=0,
                                    max=359,
                                    value=45,
                                    size="sm"
                                ),
                            ], width=6),
                        ], className="mb-2"),
                        
                        dbc.Row([
                            dbc.Col([
                                dbc.Button(
                                    "Add Custom Occlusion",
                                    id="add-custom-occlusion-btn",
                                    color="warning",
                                    size="sm",
                                    className="w-100"
                                ),
                            ], width=6),
                            dbc.Col([
                                dbc.Button(
                                    "Clear Custom",
                                    id="clear-custom-occlusion-btn",
                                    color="secondary",
                                    size="sm",
                                    className="w-100"
                                ),
                            ], width=6),
                        ], className="mb-2"),
                        
                        dbc.Button(
                            "Clear All Occlusions",
                            id="clear-all-occlusions-btn",
                            color="danger",
                            size="sm",
                            className="w-100 mb-2"
                        ),
                        
                        # Random occlusion control
                        dbc.Button(
                            "Random Occlusion (OFF)",
                            id="random-occlusion-btn",
                            color="info",
                            size="sm",
                            className="w-100 mb-2"
                        ),
                        
                        # Status display
                        html.Div(id="occlusion-status", className="small text-info"),
                    ]),
                    
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
                    dbc.Button("Random Walk (OFF)", 
                             id="random-walk-btn", 
                             color="secondary", 
                             className="mt-2 w-100"),
                    
                    html.Hr(),
                    
                    # Robot status
                    html.H6("Robot Status"),
                    html.Div(id="robot-status", className="small"),
                ])
            ])
        ], width=3),
        
        # Visualization Panel (Map + LiDAR)
        dbc.Col([
            # Map visualization
            dbc.Card([
                dbc.CardHeader("Simulation Visualization"),
                dbc.CardBody([
                    dcc.Graph(id="map-plot", style={"height": "400px"}),
                ])
            ], className="mb-3"),
            
            # LiDAR visualization
            dbc.Card([
                dbc.CardHeader("LiDAR Data"),
                dbc.CardBody([
                    dcc.Graph(id="lidar-plot", style={"height": "400px"}),
                ])
            ])
        ], width=9),
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
     Output('trustworthiness-status', 'children'),
     Output('trustworthiness-status', 'color'),
     Output('robot-status', 'children'),
     Output('occlusion-status', 'children')],
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
        height=350,
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
        height=350
    )
    
    # Status updates
    lidar_status = "OCCLUDED" if sim.lidar_occluded else "NORMAL"
    lidar_color = "danger" if sim.lidar_occluded else "success"
    
    nav_mode = "STANDARD" if sim.standard_navigation else "SPIN_CONFIG"
    nav_color = "primary" if sim.standard_navigation else "warning"
    
    trustworthiness_status = "TRUSTED" if sim.trustworthiness_status else "UNTRUSTED"
    trustworthiness_color = "success" if sim.trustworthiness_status else "danger"
    
    robot_status = html.Div([
        html.P(f"Position: ({sim.position[0]:.2f}, {sim.position[1]:.2f})"),
        html.P(f"Heading: {np.degrees(sim.heading):.1f}°"),
        html.P(f"Angle: {sim.angle:.1f}"),
        html.P(f"Navigation: {'Active' if sim.navigation_active else 'Idle'}"),
        html.P(f"Random Walk: {'ON' if sim.random_walk_active else 'OFF'}"),
        html.P(f"Failure Action: {sim.failure_action.replace('_', ' ').title()}"),
        html.P(f"Obstacles: {len(sim.obstacles)}")
    ])
    
    # Get current occlusion status
    occlusion_status = sim.get_occlusion_status()
    
    return map_fig, lidar_fig, lidar_status, lidar_color, nav_mode, nav_color, trustworthiness_status, trustworthiness_color, robot_status, occlusion_status

# Callback for LiDAR toggle
@app.callback(
    Output('toggle-lidar-btn', 'children'),
    [Input('toggle-lidar-btn', 'n_clicks')]
)
def toggle_lidar(n_clicks):
    if n_clicks:
        sim.lidar_occluded = not sim.lidar_occluded
        logging.info(f"Lidar occlusion toggled: {'ON' if sim.lidar_occluded else 'OFF'}")
    return f"Toggle LiDAR Occlusion ({'ON' if sim.lidar_occluded else 'OFF'})"

# Callback for navigation mode toggle
@app.callback(
    Output('toggle-nav-btn', 'children'),
    [Input('toggle-nav-btn', 'n_clicks')]
)
def toggle_nav_mode(n_clicks):
    if n_clicks:
        sim.standard_navigation = not sim.standard_navigation
        logging.info(f"Navigation mode toggled: {'STANDARD' if sim.standard_navigation else 'SPIN_CONFIG'}")
    return "Toggle Navigation Mode"

# Callback for stopping navigation
@app.callback(
    Output('stop-nav-btn', 'children'),
    [Input('stop-nav-btn', 'n_clicks')]
)
def stop_navigation(n_clicks):
    if n_clicks:
        sim.stop_navigation()
        logging.info("Navigation stopped by user.")
    return "Stop Navigation"

# Callback for trustworthiness toggle (for testing)
@app.callback(
    Output('toggle-trust-btn', 'children'),
    [Input('toggle-trust-btn', 'n_clicks')]
)
def toggle_trustworthiness(n_clicks):
    if n_clicks:
        sim.trustworthiness_status = not sim.trustworthiness_status
        status_text = "TRUSTED" if sim.trustworthiness_status else "UNTRUSTED"
        logging.info(f"Trustworthiness manually toggled to: {status_text}")
    return "Toggle Trustworthiness"

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
        logging.info(f"Manual navigation requested to ({target_x}, {target_y})")
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
        logging.info(f"Map click navigation requested to ({point['x']}, {point['y']})")
        # Start navigation in a separate thread
        nav_thread = threading.Thread(target=sim.navigate_to, args=(trajectory,))
        nav_thread.daemon = True
        nav_thread.start()
        return {'x': point['x'], 'y': point['y']}
    return {}

# New callback for random walk
@app.callback(
    Output('random-walk-btn', 'children'),
    [Input('random-walk-btn', 'n_clicks')]
)
def start_stop_random_walk(n_clicks):
    if n_clicks:
        if sim.random_walk_active:
            sim.stop_navigation()
            return "Random Walk (OFF)"
        else:
            # stop any current navigation and start random walk
            if sim.navigation_active:
                sim.stop_navigation()
                time.sleep(0.1)
            walker_thread = threading.Thread(target=sim.random_walk_loop)
            walker_thread.daemon = True
            walker_thread.start()
            return "Random Walk (ON)"
    return "Random Walk (ON)" if sim.random_walk_active else "Random Walk (OFF)"

# Callback for failure action dropdown
@app.callback(
    Output('failure-action-dropdown', 'value'),
    [Input('failure-action-dropdown', 'value')]
)
def update_failure_action(selected_value):
    if selected_value:
        sim.failure_action = selected_value
        logging.info(f"Failure action updated to: {selected_value}")
    return selected_value

# MQTT message handlers
def on_spin_config_message(message):
    try:
        payload = json.loads(message)
        if payload.get("commands"):
            plan = payload.get("commands")[0]
            duration = plan.get("duration")
            if duration == 0.0:
                sim.standard_navigation = True
            else:
                # Check trustworthiness status and apply failure action
                if not sim.trustworthiness_status:
                    if sim.failure_action == "apply_adaptation":
                        sim.standard_navigation = False
                        sim.angle = plan.get("omega", 90)
                        logging.info(f"Trustworthiness failed - applying adaptation: duration={duration}, omega={plan.get('omega', 90)}")
                    elif sim.failure_action == "stop_robot":
                        sim.stop_navigation()
                        logging.info("Trustworthiness failed - stopping robot")
                        return
                    elif sim.failure_action == "continue_standard":
                        sim.standard_navigation = True
                        logging.info("Trustworthiness failed - continuing standard navigation")
                        return
                else:
                    sim.standard_navigation = False
                    sim.angle = plan.get("omega", 90)
                    logging.info(f"Received spin config: duration={duration}, omega={plan.get('omega', 90)}")
    except json.JSONDecodeError:
        sim.standard_navigation = True
        logging.error("Invalid JSON in spin config message received.")

def on_trustworthiness_message(message):
    try:
        payload = json.loads(message)
        if "Bool" in payload:
            old_status = sim.trustworthiness_status
            sim.trustworthiness_status = payload["Bool"]
            if old_status != sim.trustworthiness_status:
                status_text = "TRUSTED" if sim.trustworthiness_status else "UNTRUSTED"
                logging.info(f"Trustworthiness status changed to: {status_text}")
        # If status changed from TRUSTED to UNTRUSTED, apply failure action
        if old_status and not sim.trustworthiness_status:
            if sim.failure_action == "stop_robot":
                sim.stop_navigation()
                logging.info("Trustworthiness failed - stopping robot")
            elif sim.failure_action == "continue_standard":
                sim.standard_navigation = True
                logging.info("Trustworthiness failed - continuing standard navigation")
    except json.JSONDecodeError:
        logging.error("Invalid JSON in trustworthiness message received.")

# Callbacks for selective LiDAR occlusion
@app.callback(
    Output('lidar-sector-checklist', 'value'),
    [Input('lidar-sector-checklist', 'value')]
)
def update_sector_occlusion_1(selected_sectors):
    # Update the first set of sectors
    for sector in ['front', 'front_left', 'left', 'back_left']:
        sim.lidar_occlusion_sectors[sector] = sector in selected_sectors
    logging.info(f"Sector occlusion updated (group 1): {selected_sectors}")
    return selected_sectors

@app.callback(
    Output('lidar-sector-checklist-2', 'value'),
    [Input('lidar-sector-checklist-2', 'value')]
)
def update_sector_occlusion_2(selected_sectors):
    # Update the second set of sectors
    for sector in ['back', 'back_right', 'right']:
        sim.lidar_occlusion_sectors[sector] = sector in selected_sectors
    logging.info(f"Sector occlusion updated (group 2): {selected_sectors}")
    return selected_sectors

@app.callback(
    Output('custom-start-angle', 'value'),
    [Input('add-custom-occlusion-btn', 'n_clicks')],
    [State('custom-start-angle', 'value'),
     State('custom-end-angle', 'value')]
)
def add_custom_occlusion(n_clicks, start_angle, end_angle):
    if n_clicks and start_angle is not None and end_angle is not None:
        sim.add_custom_occlusion(start_angle, end_angle)
        return 0  # Reset the start angle input
    return start_angle

@app.callback(
    Output('custom-end-angle', 'value'),
    [Input('add-custom-occlusion-btn', 'n_clicks')],
    [State('custom-start-angle', 'value'),
     State('custom-end-angle', 'value')]
)
def reset_custom_end_angle(n_clicks, start_angle, end_angle):
    if n_clicks and start_angle is not None and end_angle is not None:
        return 45  # Reset the end angle input
    return end_angle

@app.callback(
    Output('add-custom-occlusion-btn', 'children'),
    [Input('clear-custom-occlusion-btn', 'n_clicks')]
)
def clear_custom_occlusions(n_clicks):
    if n_clicks:
        sim.clear_custom_occlusions()
    return "Add Custom Occlusion"

@app.callback(
    Output('clear-all-occlusions-btn', 'children'),
    [Input('clear-all-occlusions-btn', 'n_clicks')]
)
def clear_all_occlusions(n_clicks):
    if n_clicks:
        sim.clear_all_occlusions()
        # Also reset the checkboxes
        return "Clear All Occlusions"
    return "Clear All Occlusions"

@app.callback(
    [Output('lidar-sector-checklist', 'value', allow_duplicate=True),
     Output('lidar-sector-checklist-2', 'value', allow_duplicate=True)],
    [Input('clear-all-occlusions-btn', 'n_clicks')],
    prevent_initial_call=True
)
def reset_checkboxes_on_clear_all(n_clicks):
    if n_clicks:
        return [], []
    return [], []

# Callback for random occlusion
@app.callback(
    Output('random-occlusion-btn', 'children'),
    [Input('random-occlusion-btn', 'n_clicks')]
)
def toggle_random_occlusion(n_clicks):
    if n_clicks:
        if sim.random_occlusion_active:
            sim.stop_random_occlusion()
            return "Random Occlusion (OFF)"
        else:
            sim.start_random_occlusion()
            return "Random Occlusion (ON)"
    return "Random Occlusion (OFF)"

@app.callback(
    Output('random-occlusion-btn', 'color'),
    [Input('random-occlusion-btn', 'n_clicks')]
)
def update_random_occlusion_color(n_clicks):
    if n_clicks and sim.random_occlusion_active:
        return "success"
    return "info"

# Initialize MQTT client
def initialize_client():
    global client
    try:
        # Get Redis configuration from environment variables
        redis_host = os.getenv('REDIS_HOST', 'localhost')
        redis_port = int(os.getenv('REDIS_PORT', '6379'))
        client = CommunicationManager({
            "protocol": "redis", 
            "host": redis_host, 
            "port": redis_port
        })
        client.subscribe(SPIN_CONFIG_TOPIC, callback=on_spin_config_message)
        client.subscribe(TRUSTWORTHINESS_TOPIC, callback=on_trustworthiness_message)
        client.start()
        sim.client = client
        logging.info(f"Connected to Redis at {redis_host}:{redis_port}")
        logging.info(f"Subscribed to topics: {SPIN_CONFIG_TOPIC}, {TRUSTWORTHINESS_TOPIC}")
    except Exception as e:
        logging.warning(f"Failed to initialize communication client: {e}")
        logging.info("Simulator will run without external communication")
        client = None

if __name__ == "__main__":
    # Get configuration from environment variables
    dash_host = os.getenv('DASH_HOST', '0.0.0.0')
    dash_port = int(os.getenv('DASH_PORT', '8050'))
    dash_debug = os.getenv('DASH_DEBUG', 'False').lower() == 'true'
    # Initialize MQTT client
    initialize_client()
    logging.info(f"Starting TurtleBot Dash Simulator")
    logging.info(f"Dashboard will be available at: http://{dash_host}:{dash_port}")
    logging.info(f"Debug mode: {dash_debug}")
    try:
        # Run the Dash app
        app.run(debug=dash_debug, host=dash_host, port=dash_port)
    except Exception as e:
        logging.error(f"Failed to start server: {e}")
