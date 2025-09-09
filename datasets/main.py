import pandas as pd
import numpy as np
import json
from fractions import Fraction
from lidarocclusion.masks import BoolLidarMask, ProbLidarMask
from lidarocclusion.sliding_lidar_masks import sliding_lidar_mask, sliding_prob_lidar_mask
import logging
from typing import List, Dict, Any
import ast

# Configuration parameters (from Analysis.py)
OCCLUSION_THRESHOLD = 0.3
SLIDING_WINDOW_SIZE = 3
OCCLUSION_SENSITIVITY = Fraction(1, 48)
REPLANNING_SENSITIVITY = Fraction(1, 48)

class LiDARProcessor:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO)
        self._scans = []
        self.sliding_prob_masks_generator = None
        print("LiDARProcessor initialized.")
        
    def lidar_mask_from_scan(self, scan_ranges: List[float]) -> BoolLidarMask:
        """Convert scan ranges to BoolLidarMask (from Analysis.py)"""
        scan_ranges = np.array(scan_ranges)
        return BoolLidarMask(
            (scan_ranges != np.inf) & (scan_ranges != -np.inf),
            base_angle=Fraction(2, len(scan_ranges)),
        )
    
    def setup_sliding_masks_generator(self):
        """Setup the sliding probability masks generator"""
        def scans():
            for scan in self._scans:
                yield scan

        def raw_lidar_masks():
            for scan in scans():
                yield self.lidar_mask_from_scan(scan)

        self.sliding_prob_masks_generator = sliding_prob_lidar_mask(
            raw_lidar_masks(),
            window_size=SLIDING_WINDOW_SIZE,
        )
    
    def analyze_scan(self, scan_ranges: List[float], planned_mask_json: str = None) -> Dict[str, Any]:
        print(f"Analyzing scan: {scan_ranges[:5]}... (total {len(scan_ranges)})")
        if planned_mask_json:
            print(f"Planned mask JSON: {planned_mask_json}")
        """
        Analyze a single LiDAR scan for anomalies
        Returns: dict with anomaly status, mask, and analysis results
        """
        try:
            # Add scan to buffer
            scan_dict = {"ranges": scan_ranges}
            self._scans.append(scan_dict)
            print(f"Scan buffer size: {len(self._scans)}")
            
            # If we don't have enough scans for sliding window, return no anomaly
            if len(self._scans) < SLIDING_WINDOW_SIZE:
                print("Insufficient scans for sliding window.")
                return {
                    "anomaly": "no",
                    "lidar_mask": "[]",
                    "prob_mask": "[]",
                    "mask_distance": 0.0,
                    "reason": "insufficient_data"
                }
            
            # Setup generator if not done
            if self.sliding_prob_masks_generator is None:
                print("Setting up sliding probability masks generator.")
                self.setup_sliding_masks_generator()
            
            # Get probability mask
            prob_lidar_mask = next(self.sliding_prob_masks_generator)
            print(f"Probability mask (first 5): {prob_lidar_mask.mask[:5] if hasattr(prob_lidar_mask, 'mask') else str(prob_lidar_mask)[:50]}")
            prob_lidar_mask = prob_lidar_mask.rotate(-Fraction(1, 2))
            
            # Convert to boolean mask
            lidar_mask = (prob_lidar_mask >= OCCLUSION_THRESHOLD)
            print(f"Boolean mask (first 5): {lidar_mask.mask[:5] if hasattr(lidar_mask, 'mask') else str(lidar_mask)[:50]}")
            
            # Apply weakening and strengthening (from Analysis.py)
            lidar_mask = lidar_mask.weaken(OCCLUSION_SENSITIVITY)
            lidar_mask = lidar_mask.weaken(-OCCLUSION_SENSITIVITY)
            lidar_mask = lidar_mask.strengthen(OCCLUSION_SENSITIVITY)
            lidar_mask = lidar_mask.strengthen(-OCCLUSION_SENSITIVITY)
            
            # Ignore region behind robot
            ignore_lidar_region = BoolLidarMask(
                [(3 * np.pi / 4, 5 * np.pi / 4)],
                lidar_mask.base_angle,
            )
            lidar_mask_reduced = lidar_mask | ignore_lidar_region
            
            # Get planned mask or create empty one
            if planned_mask_json and planned_mask_json != "":
                try:
                    planned_lidar_mask = BoolLidarMask.from_json(planned_mask_json)
                except:
                    planned_lidar_mask = BoolLidarMask([], Fraction(2, len(scan_ranges)))
            else:
                planned_lidar_mask = BoolLidarMask([], Fraction(2, len(scan_ranges)))
            
            # Check for anomaly
            mask_distance = lidar_mask.dist(planned_lidar_mask)
            print(f"Mask distance: {mask_distance}")
            is_anomaly = mask_distance > REPLANNING_SENSITIVITY
            print(f"Anomaly detected: {is_anomaly}")
            
            # Keep only recent scans
            if len(self._scans) > SLIDING_WINDOW_SIZE * 2:
                self._scans = self._scans[-SLIDING_WINDOW_SIZE:]
            
            return {
                "anomaly": "yes" if is_anomaly else "no",
                "lidar_mask": lidar_mask.to_json(),
                "prob_mask": prob_lidar_mask.to_json() if hasattr(prob_lidar_mask, 'to_json') else str(prob_lidar_mask),
                "mask_distance": float(mask_distance),
                "reason": "occlusion_detected" if is_anomaly else "normal"
            }
            
        except Exception as e:
            self.logger.error(f"Error analyzing scan: {e}")
            return {
                "anomaly": "error",
                "lidar_mask": "[]",
                "prob_mask": "[]",
                "mask_distance": 0.0,
                "reason": str(e)
            }

def generate_simple_plan(anomaly_detected: bool, lidar_mask_json: str) -> Dict[str, Any]:
    """
    Generate a simple navigation plan based on anomaly detection
    This is a simplified version - you would integrate with your Plan.py logic
    """
    if anomaly_detected:
        # Simple evasive plan
        plan = {
            "action": "evasive_maneuver",
            "commands": [
                {"type": "rotate", "angle": 30, "duration": 1.0},
                {"type": "move_forward", "distance": 0.5, "speed": 0.2},
                {"type": "rotate", "angle": -30, "duration": 1.0}
            ],
            "priority": "high",
            "reason": "anomaly_avoidance"
        }
    else:
        # Normal forward movement plan
        plan = {
            "action": "continue_forward",
            "commands": [
                {"type": "move_forward", "distance": 1.0, "speed": 0.5}
            ],
            "priority": "normal",
            "reason": "normal_operation"
        }
    
    return plan

def process_lidar_csv(input_csv_path: str, output_csv_path: str):
    """
    Process LiDAR data from CSV file and add anomaly detection, masks, and plans
    
    Expected CSV format:
    - First column: timestamp or scan_id
    - Second column: lidar_ranges (as string representation of list or comma-separated values)
    - Optional: planned_mask column
    """
    
    # Read the CSV file
    df = pd.read_csv(input_csv_path)
    
    # Initialize processor
    processor = LiDARProcessor()
    
    # Prepare output columns
    results = []
    
    print(f"Processing {len(df)} LiDAR scans from {input_csv_path}...")
    
    for idx, row in df.iterrows():
        print(f"\n--- Processing row {idx} ---")
        try:
            # Parse LiDAR ranges (handle different formats)
            lidar_ranges_str = str(row.iloc[1])  # Second column
            print(f"Raw lidar_ranges_str: {lidar_ranges_str[:50]}...")
            # Try different parsing methods
            if lidar_ranges_str.startswith('[') and lidar_ranges_str.endswith(']'):
                # List format: [1.0, 2.0, 3.0, ...]
                lidar_ranges = ast.literal_eval(lidar_ranges_str)
            else:
                # Comma-separated format: 1.0,2.0,3.0,...
                lidar_ranges = [float(x.strip()) for x in lidar_ranges_str.split(',')]
            print(f"Parsed lidar_ranges (first 5): {lidar_ranges[:5]}")
            
            # Get planned mask if available
            planned_mask = ""
            if len(row) > 2 and 'planned_mask' in df.columns:
                planned_mask = str(row['planned_mask']) if pd.notna(row['planned_mask']) else ""
                print(f"Planned mask for row {idx}: {planned_mask}")
            
            # Analyze the scan
            analysis_result = processor.analyze_scan(lidar_ranges, planned_mask)
            print(f"Analysis result: {analysis_result}")
            
            # Generate plan based on analysis
            plan = generate_simple_plan(
                analysis_result["anomaly"] == "yes",
                analysis_result["lidar_mask"]
            )
            print(f"Generated plan: {plan}")
            
            # Store results
            result_row = {
                'scan_id': row.iloc[0],  # First column (timestamp/id)
                'lidar_ranges': lidar_ranges_str,
                'anomaly_detected': analysis_result["anomaly"],
                'lidar_mask': analysis_result["lidar_mask"],
                'prob_mask': analysis_result["prob_mask"],
                'mask_distance': analysis_result["mask_distance"],
                'analysis_reason': analysis_result["reason"],
                'generated_plan': json.dumps(plan),
                'plan_action': plan["action"],
                'plan_priority': plan["priority"]
            }
            
            results.append(result_row)
            
            if (idx + 1) % 100 == 0:
                print(f"Processed {idx + 1}/{len(df)} scans")
                
        except Exception as e:
            print(f"Error processing row {idx}: {e}")
            # Add error row
            result_row = {
                'scan_id': row.iloc[0] if len(row) > 0 else idx,
                'lidar_ranges': str(row.iloc[1]) if len(row) > 1 else "",
                'anomaly_detected': "error",
                'lidar_mask': "[]",
                'prob_mask': "[]",
                'mask_distance': 0.0,
                'analysis_reason': str(e),
                'generated_plan': json.dumps({"action": "error", "commands": []}),
                'plan_action': "error",
                'plan_priority': "high"
            }
            results.append(result_row)
    
    # Create output DataFrame
    output_df = pd.DataFrame(results)
    
    # Save to CSV
    output_df.to_csv(output_csv_path, index=False)
    
    print(f"Processing complete! Results saved to {output_csv_path}")
    
    # Print summary statistics
    anomaly_count = len(output_df[output_df['anomaly_detected'] == 'yes'])
    normal_count = len(output_df[output_df['anomaly_detected'] == 'no'])
    error_count = len(output_df[output_df['anomaly_detected'] == 'error'])
    
    print(f"\nSummary:")
    print(f"Total scans: {len(output_df)}")
    print(f"Anomalies detected: {anomaly_count} ({anomaly_count/len(output_df)*100:.1f}%)")
    print(f"Normal scans: {normal_count} ({normal_count/len(output_df)*100:.1f}%)")
    print(f"Errors: {error_count} ({error_count/len(output_df)*100:.1f}%)")

def create_sample_csv(filename: str, num_scans: int = 100):
    """Create a sample CSV file for testing"""
    data = []
    
    for i in range(num_scans):
        # Generate sample LiDAR data (360 points)
        ranges = []
        for j in range(360):
            angle = j * np.pi / 180
            # Normal distance with some variation
            base_distance = 5.0 + 2.0 * np.sin(angle * 3)
            noise = np.random.normal(0, 0.1)
            
            # Add some anomalies randomly
            if np.random.random() < 0.1:  # 10% chance of anomaly
                distance = base_distance * 0.3  # Much closer object
            else:
                distance = base_distance + noise
            
            ranges.append(max(0.1, distance))  # Ensure positive distance
        
        data.append({
            'timestamp': i,
            'lidar_ranges': str(ranges)
        })
    
    df = pd.DataFrame(data)
    df.to_csv(filename, index=False)
    print(f"Sample CSV created: {filename}")

if __name__ == "__main__":
    # Example usage
    
    # Create sample data if needed
    sample_file = "lidar_data.csv"
    create_sample_csv(sample_file, 50)
    
    # Process the data
    output_file = "processed_lidar_data.csv"
    process_lidar_csv(sample_file, output_file)
    
    print(f"\nOutput CSV columns:")
    output_df = pd.read_csv(output_file)
    for col in output_df.columns:
        print(f"- {col}")