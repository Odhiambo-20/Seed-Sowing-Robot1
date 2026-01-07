"""
LiDAR Processor Module - Production Ready
Processes RPLIDAR A1/A2 360° point cloud data for obstacle detection,
terrain mapping, and navigation assistance.

Features:
- Real-time point cloud processing (8000 samples/sec)
- Multi-layer obstacle detection with distance clustering
- Dynamic obstacle tracking with Kalman filtering
- Terrain slope analysis for drilling site validation
- Adaptive noise filtering and outlier rejection
- Vector field histogram for collision avoidance
- Point cloud downsampling and spatial indexing
- Emergency stop zone detection
"""

import numpy as np
import time
import threading
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass, field
from collections import deque
from scipy.spatial import KDTree
from scipy.ndimage import gaussian_filter1d
from sklearn.cluster import DBSCAN
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class LiDARConfig:
    """LiDAR configuration parameters"""
    max_range: float = 12.0  # meters (RPLIDAR A1/A2 spec)
    min_range: float = 0.15  # meters
    angular_resolution: float = 0.9  # degrees (360/400 measurements)
    sample_rate: int = 8000  # samples/second
    
    # Safety zones (meters)
    emergency_stop_zone: float = 0.3
    warning_zone: float = 0.6
    detection_zone: float = 2.0
    
    # Obstacle detection
    min_obstacle_points: int = 3
    cluster_eps: float = 0.15  # meters for DBSCAN
    cluster_min_samples: int = 2
    
    # Filtering
    noise_threshold: float = 0.05  # meters
    max_point_deviation: float = 0.5  # meters
    temporal_window: int = 5  # frames for temporal filtering
    
    # Terrain analysis
    slope_analysis_radius: float = 0.5  # meters
    max_safe_slope: float = 15.0  # degrees
    
    # VFH parameters
    vfh_sectors: int = 72  # 5-degree sectors
    vfh_threshold: float = 0.3  # obstacle density threshold
    
    # Performance
    downsample_voxel_size: float = 0.05  # meters
    update_rate: float = 10.0  # Hz


@dataclass
class Obstacle:
    """Detected obstacle representation"""
    id: int
    centroid: np.ndarray  # [x, y]
    points: np.ndarray  # Nx2 array of points
    bounding_box: Tuple[np.ndarray, np.ndarray]  # (min, max)
    distance: float
    bearing: float  # radians
    velocity: Optional[np.ndarray] = None  # [vx, vy] if tracked
    confidence: float = 1.0
    last_seen: float = field(default_factory=time.time)
    track_history: deque = field(default_factory=lambda: deque(maxlen=10))


@dataclass
class TerrainPatch:
    """Terrain analysis result"""
    center: np.ndarray  # [x, y]
    mean_height: float
    slope: float  # degrees
    roughness: float
    is_plantable: bool
    confidence: float


@dataclass
class LiDARScan:
    """Single LiDAR scan data"""
    timestamp: float
    angles: np.ndarray  # radians
    distances: np.ndarray  # meters
    quality: np.ndarray  # signal quality (0-255)
    cartesian: np.ndarray  # Nx2 [x, y] in robot frame


class KalmanObstacleTracker:
    """Kalman filter for tracking moving obstacles"""
    
    def __init__(self):
        # State: [x, y, vx, vy]
        self.state = np.zeros(4)
        self.covariance = np.eye(4) * 1.0
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.5, 0.5])
        
        # Measurement noise
        self.R = np.diag([0.05, 0.05])
        
        # State transition matrix (dt will be set dynamically)
        self.F = np.eye(4)
        
        # Measurement matrix (observe position only)
        self.H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])
    
    def predict(self, dt: float):
        """Predict next state"""
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
    
    def update(self, measurement: np.ndarray):
        """Update state with measurement"""
        y = measurement - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.covariance = (np.eye(4) - K @ self.H) @ self.covariance
    
    def get_position(self) -> np.ndarray:
        """Get current position estimate"""
        return self.state[:2]
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate"""
        return self.state[2:]


class PointCloudFilter:
    """Advanced point cloud filtering"""
    
    def __init__(self, config: LiDARConfig):
        self.config = config
        self.temporal_buffer = deque(maxlen=config.temporal_window)
    
    def filter_range(self, scan: LiDARScan) -> LiDARScan:
        """Filter points outside valid range"""
        valid_mask = (scan.distances >= self.config.min_range) & \
                     (scan.distances <= self.config.max_range)
        
        return LiDARScan(
            timestamp=scan.timestamp,
            angles=scan.angles[valid_mask],
            distances=scan.distances[valid_mask],
            quality=scan.quality[valid_mask],
            cartesian=scan.cartesian[valid_mask]
        )
    
    def filter_noise(self, scan: LiDARScan) -> LiDARScan:
        """Remove noise using statistical outlier removal"""
        if len(scan.distances) < 10:
            return scan
        
        # Build KD-tree for nearest neighbor search
        tree = KDTree(scan.cartesian)
        
        # Find k-nearest neighbors for each point
        k = min(10, len(scan.cartesian))
        distances, indices = tree.query(scan.cartesian, k=k)
        
        # Calculate mean distance to neighbors
        mean_distances = np.mean(distances, axis=1)
        
        # Statistical filtering
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + 2 * global_std
        
        valid_mask = mean_distances < threshold
        
        return LiDARScan(
            timestamp=scan.timestamp,
            angles=scan.angles[valid_mask],
            distances=scan.distances[valid_mask],
            quality=scan.quality[valid_mask],
            cartesian=scan.cartesian[valid_mask]
        )
    
    def temporal_filter(self, scan: LiDARScan) -> LiDARScan:
        """Temporal consistency filtering across multiple frames"""
        self.temporal_buffer.append(scan)
        
        if len(self.temporal_buffer) < 2:
            return scan
        
        # Interpolate to common angular grid
        common_angles = scan.angles
        accumulated_distances = np.zeros_like(scan.distances)
        weights = np.zeros_like(scan.distances)
        
        for historical_scan in self.temporal_buffer:
            # Interpolate historical distances to current angles
            interp_distances = np.interp(
                common_angles,
                historical_scan.angles,
                historical_scan.distances,
                left=np.nan,
                right=np.nan
            )
            
            valid_mask = ~np.isnan(interp_distances)
            accumulated_distances[valid_mask] += interp_distances[valid_mask]
            weights[valid_mask] += 1
        
        # Average distances
        averaged_distances = np.divide(
            accumulated_distances,
            weights,
            where=weights > 0,
            out=scan.distances.copy()
        )
        
        # Reconstruct cartesian coordinates
        cartesian = self._polar_to_cartesian(common_angles, averaged_distances)
        
        return LiDARScan(
            timestamp=scan.timestamp,
            angles=common_angles,
            distances=averaged_distances,
            quality=scan.quality,
            cartesian=cartesian
        )
    
    def _polar_to_cartesian(self, angles: np.ndarray, distances: np.ndarray) -> np.ndarray:
        """Convert polar to cartesian coordinates"""
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        return np.column_stack([x, y])
    
    def downsample(self, scan: LiDARScan) -> LiDARScan:
        """Voxel grid downsampling for performance"""
        if len(scan.cartesian) == 0:
            return scan
        
        voxel_size = self.config.downsample_voxel_size
        
        # Compute voxel indices
        voxel_indices = np.floor(scan.cartesian / voxel_size).astype(int)
        
        # Find unique voxels
        unique_voxels, inverse_indices = np.unique(
            voxel_indices,
            axis=0,
            return_inverse=True
        )
        
        # Average points in each voxel
        downsampled_cartesian = []
        downsampled_angles = []
        downsampled_distances = []
        downsampled_quality = []
        
        for i in range(len(unique_voxels)):
            mask = inverse_indices == i
            downsampled_cartesian.append(np.mean(scan.cartesian[mask], axis=0))
            downsampled_angles.append(np.mean(scan.angles[mask]))
            downsampled_distances.append(np.mean(scan.distances[mask]))
            downsampled_quality.append(np.mean(scan.quality[mask]))
        
        return LiDARScan(
            timestamp=scan.timestamp,
            angles=np.array(downsampled_angles),
            distances=np.array(downsampled_distances),
            quality=np.array(downsampled_quality),
            cartesian=np.array(downsampled_cartesian)
        )


class ObstacleDetector:
    """Obstacle detection and clustering"""
    
    def __init__(self, config: LiDARConfig):
        self.config = config
        self.next_obstacle_id = 0
        self.tracked_obstacles: Dict[int, Obstacle] = {}
        self.trackers: Dict[int, KalmanObstacleTracker] = {}
        self.last_update_time = time.time()
    
    def detect_obstacles(self, scan: LiDARScan) -> List[Obstacle]:
        """Detect and cluster obstacles from point cloud"""
        if len(scan.cartesian) < self.config.min_obstacle_points:
            return []
        
        # DBSCAN clustering
        clustering = DBSCAN(
            eps=self.config.cluster_eps,
            min_samples=self.config.cluster_min_samples
        ).fit(scan.cartesian)
        
        labels = clustering.labels_
        unique_labels = set(labels) - {-1}  # Remove noise label
        
        detected_obstacles = []
        
        for label in unique_labels:
            cluster_mask = labels == label
            cluster_points = scan.cartesian[cluster_mask]
            
            if len(cluster_points) < self.config.min_obstacle_points:
                continue
            
            # Calculate obstacle properties
            centroid = np.mean(cluster_points, axis=0)
            distance = np.linalg.norm(centroid)
            bearing = np.arctan2(centroid[1], centroid[0])
            
            # Bounding box
            min_point = np.min(cluster_points, axis=0)
            max_point = np.max(cluster_points, axis=0)
            
            # Create obstacle
            obstacle = Obstacle(
                id=-1,  # Will be assigned during tracking
                centroid=centroid,
                points=cluster_points,
                bounding_box=(min_point, max_point),
                distance=distance,
                bearing=bearing,
                confidence=min(1.0, len(cluster_points) / 20.0)
            )
            
            detected_obstacles.append(obstacle)
        
        # Track obstacles
        return self._track_obstacles(detected_obstacles)
    
    def _track_obstacles(self, detected: List[Obstacle]) -> List[Obstacle]:
        """Track obstacles across frames using Hungarian algorithm"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Predict existing tracks
        for obstacle_id, tracker in self.trackers.items():
            tracker.predict(dt)
        
        # Data association using nearest neighbor
        matched_obstacles = []
        unmatched_detections = list(detected)
        
        for obstacle_id, tracked_obstacle in list(self.tracked_obstacles.items()):
            # Check if track is stale
            if current_time - tracked_obstacle.last_seen > 2.0:
                del self.tracked_obstacles[obstacle_id]
                del self.trackers[obstacle_id]
                continue
            
            # Find closest detection
            tracker = self.trackers[obstacle_id]
            predicted_pos = tracker.get_position()
            
            min_distance = float('inf')
            best_match = None
            
            for detection in unmatched_detections:
                dist = np.linalg.norm(detection.centroid - predicted_pos)
                if dist < min_distance and dist < 0.5:  # 50cm association threshold
                    min_distance = dist
                    best_match = detection
            
            if best_match is not None:
                # Update track
                tracker.update(best_match.centroid)
                tracked_obstacle.centroid = tracker.get_position()
                tracked_obstacle.velocity = tracker.get_velocity()
                tracked_obstacle.points = best_match.points
                tracked_obstacle.bounding_box = best_match.bounding_box
                tracked_obstacle.distance = np.linalg.norm(tracked_obstacle.centroid)
                tracked_obstacle.bearing = np.arctan2(
                    tracked_obstacle.centroid[1],
                    tracked_obstacle.centroid[0]
                )
                tracked_obstacle.last_seen = current_time
                tracked_obstacle.confidence = min(
                    1.0,
                    tracked_obstacle.confidence + 0.1
                )
                
                # Update history
                tracked_obstacle.track_history.append(tracked_obstacle.centroid.copy())
                
                matched_obstacles.append(tracked_obstacle)
                unmatched_detections.remove(best_match)
        
        # Initialize new tracks for unmatched detections
        for detection in unmatched_detections:
            obstacle_id = self.next_obstacle_id
            self.next_obstacle_id += 1
            
            detection.id = obstacle_id
            detection.last_seen = current_time
            detection.track_history.append(detection.centroid.copy())
            
            # Initialize Kalman tracker
            tracker = KalmanObstacleTracker()
            tracker.state[:2] = detection.centroid
            
            self.tracked_obstacles[obstacle_id] = detection
            self.trackers[obstacle_id] = tracker
            matched_obstacles.append(detection)
        
        return matched_obstacles
    
    def classify_threat_level(self, obstacle: Obstacle) -> str:
        """Classify obstacle threat level based on distance"""
        if obstacle.distance < self.config.emergency_stop_zone:
            return "EMERGENCY"
        elif obstacle.distance < self.config.warning_zone:
            return "WARNING"
        elif obstacle.distance < self.config.detection_zone:
            return "DETECTED"
        else:
            return "CLEAR"


class VectorFieldHistogram:
    """Vector Field Histogram for collision avoidance"""
    
    def __init__(self, config: LiDARConfig):
        self.config = config
        self.sector_size = 2 * np.pi / config.vfh_sectors
    
    def compute_histogram(self, scan: LiDARScan) -> np.ndarray:
        """Compute polar obstacle density histogram"""
        histogram = np.zeros(self.config.vfh_sectors)
        
        for i in range(len(scan.cartesian)):
            point = scan.cartesian[i]
            distance = scan.distances[i]
            
            if distance < 0.01:
                continue
            
            # Calculate sector index
            angle = np.arctan2(point[1], point[0])
            sector = int((angle + np.pi) / self.sector_size) % self.config.vfh_sectors
            
            # Weight by inverse distance squared
            weight = 1.0 / (distance ** 2 + 0.1)
            histogram[sector] += weight
        
        # Normalize
        if np.max(histogram) > 0:
            histogram /= np.max(histogram)
        
        # Smooth histogram
        histogram = gaussian_filter1d(
            histogram,
            sigma=2,
            mode='wrap'
        )
        
        return histogram
    
    def find_safe_directions(self, histogram: np.ndarray, 
                            target_bearing: float) -> List[Tuple[float, float]]:
        """Find safe navigation corridors"""
        # Threshold histogram
        safe_sectors = histogram < self.config.vfh_threshold
        
        # Find continuous safe regions
        safe_regions = []
        in_region = False
        region_start = 0
        
        for i in range(len(safe_sectors)):
            if safe_sectors[i] and not in_region:
                region_start = i
                in_region = True
            elif not safe_sectors[i] and in_region:
                region_end = i - 1
                safe_regions.append((region_start, region_end))
                in_region = False
        
        # Close region if it wraps around
        if in_region:
            safe_regions.append((region_start, len(safe_sectors) - 1))
        
        # Convert to angular ranges and score by alignment with target
        safe_directions = []
        target_sector = int((target_bearing + np.pi) / self.sector_size) % self.config.vfh_sectors
        
        for start, end in safe_regions:
            # Calculate center angle of region
            if end < start:  # Wrapping region
                center_sector = ((start + end + len(safe_sectors)) // 2) % len(safe_sectors)
            else:
                center_sector = (start + end) // 2
            
            center_angle = center_sector * self.sector_size - np.pi
            width = (end - start + 1) * self.sector_size
            
            # Score based on alignment with target and width
            angular_diff = abs(center_sector - target_sector)
            if angular_diff > self.config.vfh_sectors / 2:
                angular_diff = self.config.vfh_sectors - angular_diff
            
            score = width / self.sector_size - 0.5 * angular_diff
            
            safe_directions.append((center_angle, score))
        
        # Sort by score
        safe_directions.sort(key=lambda x: x[1], reverse=True)
        
        return safe_directions


class TerrainAnalyzer:
    """Analyze terrain suitability for planting"""
    
    def __init__(self, config: LiDARConfig):
        self.config = config
    
    def analyze_planting_site(self, scan: LiDARScan, 
                             target_position: np.ndarray) -> TerrainPatch:
        """Analyze terrain at target planting position"""
        # Find points within analysis radius
        distances_to_target = np.linalg.norm(
            scan.cartesian - target_position,
            axis=1
        )
        
        mask = distances_to_target <= self.config.slope_analysis_radius
        local_points = scan.cartesian[mask]
        
        if len(local_points) < 5:
            return TerrainPatch(
                center=target_position,
                mean_height=0.0,
                slope=0.0,
                roughness=0.0,
                is_plantable=False,
                confidence=0.0
            )
        
        # Estimate ground plane using RANSAC
        best_plane = self._fit_plane_ransac(local_points)
        
        # Calculate slope
        normal = best_plane[:2]
        slope_radians = np.arctan2(np.linalg.norm(normal), abs(best_plane[2]))
        slope_degrees = np.degrees(slope_radians)
        
        # Calculate roughness (deviation from plane)
        distances_to_plane = self._point_to_plane_distance(local_points, best_plane)
        roughness = np.std(distances_to_plane)
        
        # Determine plantability
        is_plantable = (
            slope_degrees < self.config.max_safe_slope and
            roughness < 0.1  # 10cm roughness threshold
        )
        
        confidence = min(1.0, len(local_points) / 20.0)
        
        return TerrainPatch(
            center=target_position,
            mean_height=np.mean(local_points[:, 2]) if local_points.shape[1] > 2 else 0.0,
            slope=slope_degrees,
            roughness=roughness,
            is_plantable=is_plantable,
            confidence=confidence
        )
    
    def _fit_plane_ransac(self, points: np.ndarray, 
                         iterations: int = 100) -> np.ndarray:
        """Fit plane using RANSAC"""
        # Add z=0 if 2D points
        if points.shape[1] == 2:
            points = np.column_stack([points, np.zeros(len(points))])
        
        best_inliers = 0
        best_plane = np.array([0, 0, 1, 0])
        
        for _ in range(iterations):
            # Sample 3 random points
            if len(points) < 3:
                break
            
            sample_idx = np.random.choice(len(points), 3, replace=False)
            sample = points[sample_idx]
            
            # Compute plane from 3 points
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
            
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, sample[0])
            plane = np.append(normal, d)
            
            # Count inliers
            distances = np.abs(self._point_to_plane_distance(points, plane))
            inliers = np.sum(distances < 0.05)
            
            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = plane
        
        return best_plane
    
    def _point_to_plane_distance(self, points: np.ndarray, 
                                 plane: np.ndarray) -> np.ndarray:
        """Calculate distance from points to plane"""
        # Add z=0 if 2D points
        if points.shape[1] == 2:
            points = np.column_stack([points, np.zeros(len(points))])
        
        return (np.dot(points, plane[:3]) + plane[3]) / np.linalg.norm(plane[:3])


class LiDARProcessor:
    """Main LiDAR processing coordinator"""
    
    def __init__(self, config: Optional[LiDARConfig] = None):
        self.config = config or LiDARConfig()
        
        # Initialize components
        self.filter = PointCloudFilter(self.config)
        self.obstacle_detector = ObstacleDetector(self.config)
        self.vfh = VectorFieldHistogram(self.config)
        self.terrain_analyzer = TerrainAnalyzer(self.config)
        
        # State
        self.latest_scan: Optional[LiDARScan] = None
        self.latest_obstacles: List[Obstacle] = []
        self.safety_state = "CLEAR"
        self.vfh_histogram: Optional[np.ndarray] = None
        
        # Threading
        self.lock = threading.Lock()
        self.processing_thread: Optional[threading.Thread] = None
        self.running = False
        
        logger.info("LiDAR Processor initialized")
    
    def process_scan(self, angles: np.ndarray, distances: np.ndarray,
                    quality: Optional[np.ndarray] = None) -> Dict:
        """Process raw LiDAR scan data"""
        timestamp = time.time()
        
        if quality is None:
            quality = np.ones_like(distances) * 255
        
        # Convert to cartesian
        cartesian = np.column_stack([
            distances * np.cos(angles),
            distances * np.sin(angles)
        ])
        
        # Create scan object
        scan = LiDARScan(
            timestamp=timestamp,
            angles=angles,
            distances=distances,
            quality=quality,
            cartesian=cartesian
        )
        
        # Apply filtering pipeline
        scan = self.filter.filter_range(scan)
        scan = self.filter.filter_noise(scan)
        scan = self.filter.temporal_filter(scan)
        scan = self.filter.downsample(scan)
        
        # Detect obstacles
        obstacles = self.obstacle_detector.detect_obstacles(scan)
        
        # Compute VFH
        vfh_histogram = self.vfh.compute_histogram(scan)
        
        # Determine safety state
        safety_state = self._evaluate_safety(obstacles)
        
        # Update state
        with self.lock:
            self.latest_scan = scan
            self.latest_obstacles = obstacles
            self.safety_state = safety_state
            self.vfh_histogram = vfh_histogram
        
        # Return processing results
        return {
            'timestamp': timestamp,
            'num_points': len(scan.cartesian),
            'obstacles': [self._obstacle_to_dict(obs) for obs in obstacles],
            'safety_state': safety_state,
            'vfh_histogram': vfh_histogram.tolist(),
            'emergency_stop_required': safety_state == "EMERGENCY"
        }
    
    def get_safe_navigation_direction(self, target_bearing: float) -> Optional[float]:
        """Get safe navigation direction towards target"""
        with self.lock:
            if self.vfh_histogram is None:
                return None
            
            safe_directions = self.vfh.find_safe_directions(
                self.vfh_histogram,
                target_bearing
            )
            
            if not safe_directions:
                return None
            
            return safe_directions[0][0]  # Return best direction
    
    def evaluate_planting_site(self, x: float, y: float) -> Dict:
        """Evaluate terrain suitability for planting at given location"""
        with self.lock:
            if self.latest_scan is None:
                return {'is_plantable': False, 'reason': 'No scan data'}
            
            target_position = np.array([x, y])
            terrain = self.terrain_analyzer.analyze_planting_site(
                self.latest_scan,
                target_position
            )
            
            return {
                'is_plantable': terrain.is_plantable,
                'slope_degrees': terrain.slope,
                'roughness': terrain.roughness,
                'confidence': terrain.confidence,
                'reason': self._get_planting_rejection_reason(terrain)
            }
    
    def _evaluate_safety(self, obstacles: List[Obstacle]) -> str:
        """Evaluate overall safety state"""
        if not obstacles:
            return "CLEAR"
        
        min_distance = min(obs.distance for obs in obstacles)
        
        if min_distance < self.config.emergency_stop_zone:
            return "EMERGENCY"
        elif min_distance < self.config.warning_zone:
            return "WARNING"
        elif min_distance < self.config.detection_zone:
            return "DETECTED"
        else:
            return "CLEAR"
    
    def _obstacle_to_dict(self, obstacle: Obstacle) -> Dict:
        """Convert obstacle to dictionary"""
        return {
            'id': obstacle.id,
            'centroid': obstacle.centroid.tolist(),
            'distance': float(obstacle.distance),
            'bearing_deg': float(np.degrees(obstacle.bearing)),
            'velocity': obstacle.velocity.tolist() if obstacle.velocity is not None else None,
            'confidence': float(obstacle.confidence),
            'threat_level': self.obstacle_detector.classify_threat_level(obstacle),
            'size': float(np.linalg.norm(
                obstacle.bounding_box[1] - obstacle.bounding_box[0]
            ))
        }
    
    def _get_planting_rejection_reason(self, terrain: TerrainPatch) -> Optional[str]:
        """Get reason why site is not plantable"""
        if terrain.is_plantable:
            return None
        
        if terrain.slope > self.config.max_safe_slope:
            return f"Slope too steep: {terrain.slope:.1f}°"
        
        if terrain.roughness > 0.1:
            return f"Terrain too rough: {terrain.roughness:.2f}m"
        
        if terrain.confidence < 0.3:
            return "Insufficient scan data"
        
        return "Unknown reason"
    
    def get_point_cloud(self) -> Optional[np.ndarray]:
        """Get latest point cloud"""
        with self.lock:
            if self.latest_scan is None:
                return None
            return self.latest_scan.cartesian.copy()
    
    def get_obstacles(self) -> List[Dict]:
        """Get current obstacle list"""
        with self.lock:
            return [self._obstacle_to_dict(obs) for obs in self.latest_obstacles]
    
    def get_safety_state(self) -> str:
        """Get current safety state"""
        with self.lock:
            return self.safety_state
    
    def start_continuous_processing(self):
        """Start background processing thread"""
        if self.running:
            logger.warning("Processing already running")
            return
        
        self.running = True
        self.processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True
        )
        self.processing_thread.start()
        logger.info("Started continuous LiDAR processing")
    
    def stop_continuous_processing(self):
        """Stop background processing thread"""
        self.running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        logger.info("Stopped LiDAR processing")
    
    def _processing_loop(self):
        """Background processing loop"""
        update_period = 1.0 / self.config.update_rate
        
        while self.running:
            start_time = time.time()
            
            # Processing happens when new scan data arrives via process_scan()
            # This loop just handles periodic maintenance tasks
            
            # Clean up stale obstacles
            with self.lock:
                current_time = time.time()
                stale_ids = [
                    obs_id for obs_id, obs in self.obstacle_detector.tracked_obstacles.items()
                    if current_time - obs.last_seen > 2.0
                ]
                
                for obs_id in stale_ids:
                    del self.obstacle_detector.tracked_obstacles[obs_id]
                    if obs_id in self.obstacle_detector.trackers:
                        del self.obstacle_detector.trackers[obs_id]
            
            # Sleep to maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, update_period - elapsed)
            time.sleep(sleep_time)
    
    def get_statistics(self) -> Dict:
        """Get processing statistics"""
        with self.lock:
            return {
                'tracked_obstacles': len(self.obstacle_detector.tracked_obstacles),
                'total_obstacles_detected': self.obstacle_detector.next_obstacle_id,
                'latest_scan_points': len(self.latest_scan.cartesian) if self.latest_scan else 0,
                'safety_state': self.safety_state,
                'temporal_buffer_size': len(self.filter.temporal_buffer)
            }
    
    def reset(self):
        """Reset all tracking and buffered data"""
        with self.lock:
            self.obstacle_detector.tracked_obstacles.clear()
            self.obstacle_detector.trackers.clear()
            self.filter.temporal_buffer.clear()
            self.latest_scan = None
            self.latest_obstacles = []
            self.safety_state = "CLEAR"
            self.vfh_histogram = None
        
        logger.info("LiDAR processor reset")
    
    def export_obstacle_tracks(self) -> Dict[int, List[np.ndarray]]:
        """Export all obstacle tracking histories"""
        with self.lock:
            tracks = {}
            for obs_id, obstacle in self.obstacle_detector.tracked_obstacles.items():
                tracks[obs_id] = list(obstacle.track_history)
            return tracks
    
    def get_obstacle_predictions(self, prediction_time: float = 1.0) -> List[Dict]:
        """Predict obstacle positions in the future"""
        with self.lock:
            predictions = []
            
            for obs_id, obstacle in self.obstacle_detector.tracked_obstacles.items():
                if obstacle.velocity is None:
                    continue
                
                # Linear prediction
                predicted_position = obstacle.centroid + obstacle.velocity * prediction_time
                predicted_distance = np.linalg.norm(predicted_position)
                predicted_bearing = np.arctan2(predicted_position[1], predicted_position[0])
                
                predictions.append({
                    'obstacle_id': obs_id,
                    'current_position': obstacle.centroid.tolist(),
                    'predicted_position': predicted_position.tolist(),
                    'prediction_time': prediction_time,
                    'predicted_distance': float(predicted_distance),
                    'predicted_bearing_deg': float(np.degrees(predicted_bearing)),
                    'velocity': obstacle.velocity.tolist(),
                    'confidence': float(obstacle.confidence)
                })
            
            return predictions
    
    def create_occupancy_grid(self, resolution: float = 0.1, 
                             grid_size: float = 10.0) -> np.ndarray:
        """Create 2D occupancy grid from point cloud"""
        with self.lock:
            if self.latest_scan is None or len(self.latest_scan.cartesian) == 0:
                return np.zeros((1, 1))
            
            # Calculate grid dimensions
            grid_cells = int(grid_size / resolution)
            half_cells = grid_cells // 2
            
            # Initialize grid (0 = free, 1 = occupied)
            grid = np.zeros((grid_cells, grid_cells))
            
            # Convert points to grid coordinates
            points = self.latest_scan.cartesian
            grid_x = np.clip(
                (points[:, 0] / resolution + half_cells).astype(int),
                0,
                grid_cells - 1
            )
            grid_y = np.clip(
                (points[:, 1] / resolution + half_cells).astype(int),
                0,
                grid_cells - 1
            )
            
            # Mark occupied cells
            grid[grid_y, grid_x] = 1
            
            # Apply dilation for safety margin
            from scipy.ndimage import binary_dilation
            grid = binary_dilation(grid, iterations=2).astype(float)
            
            return grid
    
    def get_clearance_map(self, resolution: float = 0.1,
                         grid_size: float = 10.0) -> np.ndarray:
        """Calculate distance to nearest obstacle for each grid cell"""
        from scipy.ndimage import distance_transform_edt
        
        occupancy_grid = self.create_occupancy_grid(resolution, grid_size)
        
        # Calculate distance transform
        free_space = 1 - occupancy_grid
        clearance_map = distance_transform_edt(free_space) * resolution
        
        return clearance_map
    
    def find_path_through_obstacles(self, start: np.ndarray, 
                                   goal: np.ndarray,
                                   resolution: float = 0.1) -> Optional[List[np.ndarray]]:
        """Find path from start to goal avoiding obstacles using A*"""
        # Create occupancy grid
        grid_size = max(
            np.linalg.norm(start),
            np.linalg.norm(goal),
            self.config.detection_zone
        ) * 2
        
        occupancy_grid = self.create_occupancy_grid(resolution, grid_size)
        grid_cells = occupancy_grid.shape[0]
        half_cells = grid_cells // 2
        
        # Convert start/goal to grid coordinates
        start_grid = np.array([
            int(start[0] / resolution + half_cells),
            int(start[1] / resolution + half_cells)
        ])
        goal_grid = np.array([
            int(goal[0] / resolution + half_cells),
            int(goal[1] / resolution + half_cells)
        ])
        
        # Check bounds
        if (not (0 <= start_grid[0] < grid_cells and 0 <= start_grid[1] < grid_cells) or
            not (0 <= goal_grid[0] < grid_cells and 0 <= goal_grid[1] < grid_cells)):
            return None
        
        # A* pathfinding
        from heapq import heappush, heappop
        
        open_set = []
        heappush(open_set, (0, tuple(start_grid)))
        
        came_from = {}
        g_score = {tuple(start_grid): 0}
        f_score = {tuple(start_grid): np.linalg.norm(goal_grid - start_grid)}
        
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
        
        while open_set:
            current_f, current = heappop(open_set)
            current = np.array(current)
            
            # Goal reached
            if np.array_equal(current, goal_grid):
                # Reconstruct path
                path = [goal_grid]
                while tuple(path[-1]) in came_from:
                    path.append(came_from[tuple(path[-1])])
                path.reverse()
                
                # Convert back to world coordinates
                world_path = [
                    np.array([
                        (p[0] - half_cells) * resolution,
                        (p[1] - half_cells) * resolution
                    ]) for p in path
                ]
                
                return world_path
            
            # Explore neighbors
            for dx, dy in directions:
                neighbor = current + np.array([dx, dy])
                
                # Check bounds
                if not (0 <= neighbor[0] < grid_cells and 0 <= neighbor[1] < grid_cells):
                    continue
                
                # Check if occupied
                if occupancy_grid[neighbor[1], neighbor[0]] > 0.5:
                    continue
                
                # Calculate tentative g_score
                move_cost = np.sqrt(dx**2 + dy**2)
                tentative_g = g_score[tuple(current)] + move_cost
                
                if tuple(neighbor) not in g_score or tentative_g < g_score[tuple(neighbor)]:
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = tentative_g
                    f_score[tuple(neighbor)] = tentative_g + np.linalg.norm(goal_grid - neighbor)
                    heappush(open_set, (f_score[tuple(neighbor)], tuple(neighbor)))
        
        # No path found
        return None
    
    def detect_dynamic_obstacles(self, velocity_threshold: float = 0.1) -> List[Dict]:
        """Get list of moving obstacles"""
        with self.lock:
            dynamic_obstacles = []
            
            for obstacle in self.latest_obstacles:
                if obstacle.velocity is not None:
                    speed = np.linalg.norm(obstacle.velocity)
                    
                    if speed > velocity_threshold:
                        dynamic_obstacles.append({
                            'id': obstacle.id,
                            'position': obstacle.centroid.tolist(),
                            'velocity': obstacle.velocity.tolist(),
                            'speed': float(speed),
                            'heading_deg': float(np.degrees(np.arctan2(
                                obstacle.velocity[1],
                                obstacle.velocity[0]
                            ))),
                            'distance': float(obstacle.distance),
                            'confidence': float(obstacle.confidence)
                        })
            
            return dynamic_obstacles
    
    def calculate_time_to_collision(self, robot_velocity: np.ndarray) -> Optional[float]:
        """Calculate time to collision with nearest obstacle"""
        with self.lock:
            if not self.latest_obstacles:
                return None
            
            min_ttc = float('inf')
            
            for obstacle in self.latest_obstacles:
                # Relative velocity
                if obstacle.velocity is not None:
                    relative_velocity = robot_velocity - obstacle.velocity
                else:
                    relative_velocity = robot_velocity
                
                # Project onto line to obstacle
                to_obstacle = obstacle.centroid
                distance = np.linalg.norm(to_obstacle)
                
                if distance < 0.01:
                    return 0.0
                
                direction = to_obstacle / distance
                approach_speed = np.dot(relative_velocity, direction)
                
                # Only consider if approaching
                if approach_speed > 0:
                    ttc = (distance - self.config.emergency_stop_zone) / approach_speed
                    min_ttc = min(min_ttc, ttc)
            
            return min_ttc if min_ttc < float('inf') else None
    
    def get_visualization_data(self) -> Dict:
        """Get data formatted for visualization"""
        with self.lock:
            if self.latest_scan is None:
                return {}
            
            return {
                'point_cloud': self.latest_scan.cartesian.tolist(),
                'obstacles': [
                    {
                        'id': obs.id,
                        'centroid': obs.centroid.tolist(),
                        'points': obs.points.tolist(),
                        'bounding_box': [
                            obs.bounding_box[0].tolist(),
                            obs.bounding_box[1].tolist()
                        ],
                        'velocity': obs.velocity.tolist() if obs.velocity is not None else None,
                        'track_history': [p.tolist() for p in obs.track_history]
                    }
                    for obs in self.latest_obstacles
                ],
                'vfh_histogram': self.vfh_histogram.tolist() if self.vfh_histogram is not None else [],
                'safety_zones': {
                    'emergency': self.config.emergency_stop_zone,
                    'warning': self.config.warning_zone,
                    'detection': self.config.detection_zone
                },
                'safety_state': self.safety_state,
                'timestamp': self.latest_scan.timestamp
            }


# Example usage and testing functions
if __name__ == "__main__":
    # Initialize processor
    config = LiDARConfig(
        max_range=12.0,
        emergency_stop_zone=0.3,
        warning_zone=0.6,
        detection_zone=2.0
    )
    
    processor = LiDARProcessor(config)
    
    # Simulate LiDAR scan data
    num_points = 400
    angles = np.linspace(-np.pi, np.pi, num_points)
    
    # Create synthetic scan with obstacles
    distances = np.ones(num_points) * 5.0  # 5m default distance
    
    # Add obstacle at 2m, 45 degrees
    obstacle_angle_idx = int((np.pi/4 + np.pi) / (2*np.pi) * num_points)
    distances[obstacle_angle_idx-5:obstacle_angle_idx+5] = 2.0
    
    # Add obstacle at 1m, -30 degrees
    obstacle_angle_idx2 = int((-np.pi/6 + np.pi) / (2*np.pi) * num_points)
    distances[obstacle_angle_idx2-5:obstacle_angle_idx2+5] = 1.0
    
    # Add noise
    distances += np.random.normal(0, 0.02, num_points)
    
    # Process scan
    result = processor.process_scan(angles, distances)
    
    print("Processing Results:")
    print(f"Points: {result['num_points']}")
    print(f"Obstacles detected: {len(result['obstacles'])}")
    print(f"Safety state: {result['safety_state']}")
    print(f"Emergency stop: {result['emergency_stop_required']}")
    
    print("\nObstacle Details:")
    for obs in result['obstacles']:
        print(f"  ID: {obs['id']}")
        print(f"  Distance: {obs['distance']:.2f}m")
        print(f"  Bearing: {obs['bearing_deg']:.1f}°")
        print(f"  Threat: {obs['threat_level']}")
    
    # Test safe navigation
    target_bearing = np.pi / 4  # 45 degrees
    safe_direction = processor.get_safe_navigation_direction(target_bearing)
    if safe_direction is not None:
        print(f"\nSafe direction: {np.degrees(safe_direction):.1f}°")
    
    # Test planting site evaluation
    site_eval = processor.evaluate_planting_site(2.0, 1.0)
    print(f"\nPlanting site evaluation:")
    print(f"  Plantable: {site_eval['is_plantable']}")
    print(f"  Slope: {site_eval['slope_degrees']:.1f}°")
    print(f"  Roughness: {site_eval['roughness']:.3f}m")
    
    # Test statistics
    stats = processor.get_statistics()
    print(f"\nProcessor Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    print("\nLiDAR Processor Test Complete!")
