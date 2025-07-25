#!/usr/bin/env python3
"""
Redis Log Viewer for TurtleBot Simulator
Displays logs stored in Redis with format: containername:logs
"""

import redis
import json
import argparse
import time
from datetime import datetime

def view_logs(redis_host='localhost', redis_port=6379, container_name='turtlebot_dash_simulator', follow=False, lines=50):
    """View logs from Redis"""
    try:
        # Connect to Redis
        r = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        log_key = f"{container_name}:logs"
        
        print(f"📋 Viewing logs from: {log_key}")
        print(f"🔗 Redis: {redis_host}:{redis_port}")
        print("="*80)
        
        if follow:
            print("📡 Following logs (Ctrl+C to exit)...")
            last_length = 0
            
            while True:
                try:
                    # Get current length
                    current_length = r.llen(log_key)
                    
                    if current_length > last_length:
                        # Get new logs
                        new_logs = r.lrange(log_key, 0, current_length - last_length - 1)
                        
                        for log_entry in reversed(new_logs):
                            try:
                                log_data = json.loads(log_entry)
                                timestamp = log_data.get('timestamp', 'Unknown')
                                level = log_data.get('level', 'INFO')
                                message = log_data.get('message', '')
                                container = log_data.get('container', 'unknown')
                                
                                # Format timestamp
                                try:
                                    dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                                    formatted_time = dt.strftime('%H:%M:%S')
                                except:
                                    formatted_time = timestamp
                                
                                # Color coding for levels
                                color_codes = {
                                    'INFO': '\033[92m',     # Green
                                    'WARNING': '\033[93m',  # Yellow
                                    'ERROR': '\033[91m',    # Red
                                    'DEBUG': '\033[94m'     # Blue
                                }
                                color = color_codes.get(level, '\033[0m')
                                reset_color = '\033[0m'
                                
                                print(f"[{formatted_time}] {color}[{level}]{reset_color} [{container}] {message}")
                                
                            except json.JSONDecodeError:
                                print(f"[Invalid JSON] {log_entry}")
                        
                        last_length = current_length
                    
                    time.sleep(1)
                    
                except KeyboardInterrupt:
                    print("\n👋 Stopped following logs")
                    break
        else:
            # Get recent logs
            logs = r.lrange(log_key, 0, lines - 1)
            
            if not logs:
                print(f"❌ No logs found for key: {log_key}")
                print("💡 Available keys:")
                keys = r.keys("*:logs")
                for key in keys:
                    print(f"   - {key}")
                return
            
            print(f"📊 Showing last {len(logs)} log entries:")
            print()
            
            for log_entry in reversed(logs):
                try:
                    log_data = json.loads(log_entry)
                    timestamp = log_data.get('timestamp', 'Unknown')
                    level = log_data.get('level', 'INFO')
                    message = log_data.get('message', '')
                    container = log_data.get('container', 'unknown')
                    
                    # Format timestamp
                    try:
                        dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                        formatted_time = dt.strftime('%Y-%m-%d %H:%M:%S')
                    except:
                        formatted_time = timestamp
                    
                    print(f"[{formatted_time}] [{level}] [{container}] {message}")
                    
                except json.JSONDecodeError:
                    print(f"[Invalid JSON] {log_entry}")
                    
    except redis.ConnectionError:
        print(f"❌ Could not connect to Redis at {redis_host}:{redis_port}")
        print("💡 Make sure Redis is running and accessible")
    except Exception as e:
        print(f"❌ Error: {e}")

def list_containers(redis_host='localhost', redis_port=6379):
    """List available log containers"""
    try:
        r = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        keys = r.keys("*:logs")
        
        if keys:
            print("📋 Available log containers:")
            for key in keys:
                container_name = key.replace(':logs', '')
                log_count = r.llen(key)
                print(f"   - {container_name} ({log_count} logs)")
        else:
            print("❌ No log containers found")
            
    except redis.ConnectionError:
        print(f"❌ Could not connect to Redis at {redis_host}:{redis_port}")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    parser = argparse.ArgumentParser(description="Redis Log Viewer for TurtleBot Simulator")
    parser.add_argument('--host', default='localhost', help='Redis host (default: localhost)')
    parser.add_argument('--port', type=int, default=6379, help='Redis port (default: 6379)')
    parser.add_argument('--container', default='turtlebot_dash_simulator', 
                       help='Container name (default: turtlebot_dash_simulator)')
    parser.add_argument('--lines', '-n', type=int, default=50, 
                       help='Number of lines to show (default: 50)')
    parser.add_argument('--follow', '-f', action='store_true', 
                       help='Follow logs in real-time')
    parser.add_argument('--list', '-l', action='store_true', 
                       help='List available log containers')
    
    args = parser.parse_args()
    
    if args.list:
        list_containers(args.host, args.port)
    else:
        view_logs(args.host, args.port, args.container, args.follow, args.lines)

if __name__ == "__main__":
    main()
