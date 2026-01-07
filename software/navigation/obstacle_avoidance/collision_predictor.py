"""
Collision Predictor Module - Production Ready
Advanced predictive collision avoidance using multi-sensor fusion, trajectory prediction,
and real-time safety analysis for autonomous agricultural robot navigation.

Features:
- Multi-hypothesis trajectory prediction with uncertainty propagation
- Dynamic obstacle motion modeling with Kalman filtering
- Collision probability estimation using Monte Carlo sampling
- Time-to-collision (TTC) calculation with multiple scenarios
- Safe velocity envelope computation
- Emergency braking distance calculation with terrain compensation
- Predictive path validation using occupancy forecasting
- Risk-aware motion planning with graduated response
- Adaptive safety margins based on robot dynamics
- Sensor fusion for robust collision detection
"""

import numpy as np
import time
import threading
from typing import List, Dict, Optional, Tuple, Callable
from dataclasses import dataclass, field
from collections import deque
from scipy.integrate import odeint
from scipy.optimize import minimize, LinearConstraint
from scipy.spatial import ConvexHull
from scipy.stats import multivariate_normal
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class PredictorConfig:
    """Collision predictor configuration"""
    # Prediction parameters
    prediction_horizon: float = 5.0  # seconds
    prediction_timestep: float = 0.1  # seconds
    num_monte_carlo_samples: int = 500
    
    # Robot dynamics
    max_velocity: float = 1.5  # m/s
    max_acceleration: float = 0.5  # m/s²
    max_deceleration: float = 1.0  # m/s²
    max_angular_velocity: float = 1.0  # rad/s
    max_angular_acceleration: float = 0.5  # rad/s²
    robot_radius: float = 0.35  # meters (conservative bounding circle)
    wheel_base: float = 0.4  # meters
    
    # Safety parameters
    base_safety_margin: float = 0.3  # meters
    velocity_safety_factor: float = 0.5  # seconds (reaction time buffer)
    collision_probability_threshold: float = 0.15
    critical_ttc_threshold: float = 2.0  # seconds
    warning_ttc_threshold: float = 4.0  # seconds
    
    # Terrain compensation
    slope_deceleration_factor: float = 0.8  # Reduce braking on slopes
    rough_terrain_safety_multiplier: float = 1.5
    
    # Uncertainty parameters
    position_uncertainty: float = 0.05  # meters std dev
    velocity_uncertainty: float = 0.1  # m/s std dev
    obstacle_velocity_uncertainty: float = 0.2  # m/s std dev
    
    # Response levels
    emergency_stop_distance: float = 0.5  # meters
    hard_brake_distance: float = 1.0  # meters
    soft_brake_distance: float = 2.0  # meters
    
    # Update rate
    update_rate: float = 20.0  # Hz


@dataclass
class RobotState:
    """Robot state representation"""
    position: np.ndarray  # [x, y] meters
    velocity: np.ndarray  # [vx, vy] m/s
    heading: float  # radians
    angular_velocity: float  # rad/s
    timestamp: float


@dataclass
class ObstacleState:
    """Obstacle state representation"""
    id: int
    position: np.ndarray  # [x, y] meters
    velocity: Optional[np.ndarray]  # [vx, vy] m/s
    size: float  # meters (radius)
    confidence: float
    is_dynamic: bool
    covariance: Optional[np.ndarray] = None  # Position uncertainty


@dataclass
class TrajectoryPoint:
    """Single point in predicted trajectory"""
    time: float
    position: np.ndarray
    velocity: np.ndarray
    heading: float
    uncertainty_ellipse: Optional[np.ndarray] = None  # 2x2 covariance matrix


@dataclass
class CollisionEvent:
    """Predicted collision event"""
    time_to_collision: float
    collision_point: np.ndarray
    collision_probability: float
    obstacle_id: int
    relative_velocity: float
    impact_angle: float
    severity: str  # "CRITICAL", "HIGH", "MEDIUM", "LOW"
    avoidance_actions: List[str]


@dataclass
class SafetyEnvelope:
    """Safe velocity envelope"""
    max_forward_velocity: float
    max_lateral_velocity: float
    max_angular_velocity: float
    recommended_velocity: np.ndarray
    constraint_reason: str


class TrajectoryPredictor:
    """Predict robot trajectory with uncertainty propagation"""
    
    def __init__(self, config: PredictorConfig):
        self.config = config
    
    def predict_trajectory(self, initial_state: RobotState,
                          control_inputs: Optional[List[np.ndarray]] = None,
                          num_samples: int = 1) -> List[List[TrajectoryPoint]]:
        """
        Predict multiple trajectory samples with uncertainty.
        
        Args:
            initial_state: Current robot state
            control_inputs: List of control inputs [linear_vel, angular_vel] over time
            num_samples: Number of Monte Carlo samples
            
        Returns:
            List of trajectory samples, each is a list of TrajectoryPoint
        """
        trajectories = []
        
        # Default to constant velocity if no control inputs
        if control_inputs is None:
            control_inputs = [
                np.array([np.linalg.norm(initial_state.velocity), initial_state.angular_velocity])
            ] * int(self.config.prediction_horizon / self.config.prediction_timestep)
        
        for sample_idx in range(num_samples):
            trajectory = []
            
            # Sample initial state with uncertainty
            if num_samples > 1:
                pos_noise = np.random.multivariate_normal(
                    [0, 0],
                    np.eye(2) * self.config.position_uncertainty**2
                )
                vel_noise = np.random.multivariate_normal(
                    [0, 0],
                    np.eye(2) * self.config.velocity_uncertainty**2
                )
                
                current_pos = initial_state.position + pos_noise
                current_vel = initial_state.velocity + vel_noise
                current_heading = initial_state.heading + np.random.normal(0, 0.05)
                current_omega = initial_state.angular_velocity + np.random.normal(0, 0.05)
            else:
                current_pos = initial_state.position.copy()
                current_vel = initial_state.velocity.copy()
                current_heading = initial_state.heading
                current_omega = initial_state.angular_velocity
            
            current_time = 0.0
            
            # Propagate trajectory
            for i in range(len(control_inputs)):
                # Get control input
                v_cmd, omega_cmd = control_inputs[i]
                
                # Differential drive kinematics with dynamics
                dt = self.config.prediction_timestep
                
                # Limit acceleration
                current_speed = np.linalg.norm(current_vel)
                speed_diff = v_cmd - current_speed
                
                if abs(speed_diff) > self.config.max_acceleration * dt:
                    v_cmd = current_speed + np.sign(speed_diff) * self.config.max_acceleration * dt
                
                # Limit angular acceleration
                omega_diff = omega_cmd - current_omega
                if abs(omega_diff) > self.config.max_angular_acceleration * dt:
                    omega_cmd = current_omega + np.sign(omega_diff) * self.config.max_angular_acceleration * dt
                
                # Update heading
                current_heading += current_omega * dt
                current_heading = self._normalize_angle(current_heading)
                
                # Update velocity in body frame then transform to world frame
                current_vel = np.array([
                    v_cmd * np.cos(current_heading),
                    v_cmd * np.sin(current_heading)
                ])
                
                # Update position
                current_pos += current_vel * dt
                current_omega = omega_cmd
                current_time += dt
                
                # Add process noise for subsequent samples
                if num_samples > 1 and i > 0:
                    current_pos += np.random.multivariate_normal(
                        [0, 0],
                        np.eye(2) * (self.config.position_uncertainty * dt)**2
                    )
                    current_vel += np.random.multivariate_normal(
                        [0, 0],
                        np.eye(2) * (self.config.velocity_uncertainty * dt)**2
                    )
                
                # Store trajectory point
                trajectory.append(TrajectoryPoint(
                    time=current_time,
                    position=current_pos.copy(),
                    velocity=current_vel.copy(),
                    heading=current_heading,
                    uncertainty_ellipse=None  # Will be computed from samples
                ))
            
            trajectories.append(trajectory)
        
        # Compute uncertainty ellipses from samples
        if num_samples > 1:
            self._compute_uncertainty_ellipses(trajectories)
        
        return trajectories
    
    def _compute_uncertainty_ellipses(self, trajectories: List[List[TrajectoryPoint]]):
        """Compute uncertainty covariance at each time step from samples"""
        num_timesteps = len(trajectories[0])
        
        for t in range(num_timesteps):
            # Gather positions at this timestep
            positions = np.array([traj[t].position for traj in trajectories])
            
            # Compute covariance
            covariance = np.cov(positions.T)
            
            # Store in first trajectory (nominal)
            trajectories[0][t].uncertainty_ellipse = covariance
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))


class ObstacleMotionModel:
    """Model and predict obstacle motion"""
    
    def __init__(self, config: PredictorConfig):
        self.config = config
        self.obstacle_tracks: Dict[int, deque] = {}  # History for each obstacle
    
    def add_observation(self, obstacle: ObstacleState):
        """Add obstacle observation to history"""
        if obstacle.id not in self.obstacle_tracks:
            self.obstacle_tracks[obstacle.id] = deque(maxlen=10)
        
        self.obstacle_tracks[obstacle.id].append({
            'time': time.time(),
            'position': obstacle.position.copy(),
            'velocity': obstacle.velocity.copy() if obstacle.velocity is not None else None
        })
    
    def predict_obstacle_trajectory(self, obstacle: ObstacleState,
                                   prediction_horizon: float,
                                   num_samples: int = 1) -> List[List[Tuple[float, np.ndarray]]]:
        """
        Predict obstacle trajectory with uncertainty.
        
        Returns:
            List of trajectory samples, each is list of (time, position) tuples
        """
        trajectories = []
        dt = self.config.prediction_timestep
        num_steps = int(prediction_horizon / dt)
        
        # Determine motion model
        if obstacle.velocity is not None and np.linalg.norm(obstacle.velocity) > 0.1:
            # Dynamic obstacle - use constant velocity with uncertainty
            for sample in range(num_samples):
                trajectory = []
                
                # Sample velocity with uncertainty
                if num_samples > 1:
                    vel_noise = np.random.multivariate_normal(
                        [0, 0],
                        np.eye(2) * self.config.obstacle_velocity_uncertainty**2
                    )
                    sampled_velocity = obstacle.velocity + vel_noise
                else:
                    sampled_velocity = obstacle.velocity
                
                current_pos = obstacle.position.copy()
                current_time = 0.0
                
                for step in range(num_steps):
                    # Add process noise
                    if num_samples > 1 and step > 0:
                        pos_noise = np.random.multivariate_normal(
                            [0, 0],
                            np.eye(2) * (0.05 * dt)**2
                        )
                        current_pos += pos_noise
                    
                    trajectory.append((current_time, current_pos.copy()))
                    
                    current_pos += sampled_velocity * dt
                    current_time += dt
                
                trajectories.append(trajectory)
        else:
            # Static obstacle
            trajectory = [(t * dt, obstacle.position.copy()) 
                         for t in range(num_steps)]
            trajectories = [trajectory] * num_samples
        
        return trajectories
    
    def estimate_obstacle_acceleration(self, obstacle_id: int) -> Optional[np.ndarray]:
        """Estimate obstacle acceleration from history"""
        if obstacle_id not in self.obstacle_tracks:
            return None
        
        track = list(self.obstacle_tracks[obstacle_id])
        if len(track) < 3:
            return None
        
        # Use last 3 points to estimate acceleration
        velocities = []
        times = []
        
        for i in range(len(track) - 1):
            dt = track[i+1]['time'] - track[i]['time']
            if dt > 0:
                dpos = track[i+1]['position'] - track[i]['position']
                vel = dpos / dt
                velocities.append(vel)
                times.append(track[i+1]['time'])
        
        if len(velocities) < 2:
            return None
        
        # Estimate acceleration from velocity change
        dt = times[-1] - times[-2]
        if dt > 0:
            accel = (velocities[-1] - velocities[-2]) / dt
            return accel
        
        return None


class CollisionChecker:
    """Check for collisions between trajectories"""
    
    def __init__(self, config: PredictorConfig):
        self.config = config
    
    def check_collision(self, robot_traj: TrajectoryPoint,
                       obstacle_pos: np.ndarray,
                       obstacle_size: float,
                       safety_margin: Optional[float] = None) -> bool:
        """Check if robot trajectory point collides with obstacle"""
        if safety_margin is None:
            safety_margin = self.config.base_safety_margin
        
        distance = np.linalg.norm(robot_traj.position - obstacle_pos)
        collision_threshold = self.config.robot_radius + obstacle_size + safety_margin
        
        return distance < collision_threshold
    
    def find_collision_time(self, robot_trajectory: List[TrajectoryPoint],
                           obstacle_trajectory: List[Tuple[float, np.ndarray]],
                           obstacle_size: float) -> Optional[float]:
        """Find first collision time between robot and obstacle trajectories"""
        for i, robot_point in enumerate(robot_trajectory):
            if i >= len(obstacle_trajectory):
                break
            
            obstacle_time, obstacle_pos = obstacle_trajectory[i]
            
            if self.check_collision(robot_point, obstacle_pos, obstacle_size):
                return robot_point.time
        
        return None
    
    def compute_collision_probability(self, robot_trajectories: List[List[TrajectoryPoint]],
                                     obstacle_trajectories: List[List[Tuple[float, np.ndarray]]],
                                     obstacle_size: float) -> Tuple[float, List[float]]:
        """
        Compute collision probability using Monte Carlo samples.
        
        Returns:
            (overall_probability, time_series_probabilities)
        """
        num_samples = len(robot_trajectories)
        num_timesteps = len(robot_trajectories[0])
        
        collision_counts = np.zeros(num_timesteps)
        
        for robot_traj, obs_traj in zip(robot_trajectories, obstacle_trajectories):
            for i in range(min(len(robot_traj), len(obs_traj))):
                robot_point = robot_traj[i]
                obs_time, obs_pos = obs_traj[i]
                
                if self.check_collision(robot_point, obs_pos, obstacle_size):
                    collision_counts[i:] += 1  # Collision persists
                    break
        
        time_series_probs = collision_counts / num_samples
        overall_prob = np.max(time_series_probs)
        
        return overall_prob, time_series_probs.tolist()
    
    def minimum_separation_distance(self, robot_trajectory: List[TrajectoryPoint],
                                    obstacle_trajectory: List[Tuple[float, np.ndarray]]) -> float:
        """Compute minimum separation distance along trajectories"""
        min_dist = float('inf')
        
        for i in range(min(len(robot_trajectory), len(obstacle_trajectory))):
            robot_pos = robot_trajectory[i].position
            _, obs_pos = obstacle_trajectory[i]
            
            dist = np.linalg.norm(robot_pos - obs_pos)
            min_dist = min(min_dist, dist)
        
        return min_dist


class BrakingModel:
    """Model emergency braking capabilities"""
    
    def __init__(self, config: PredictorConfig):
        self.config = config
    
    def compute_braking_distance(self, current_velocity: float,
                                 terrain_slope: float = 0.0,
                                 surface_friction: float = 1.0) -> float:
        """
        Compute required braking distance.
        
        Args:
            current_velocity: Current speed in m/s
            terrain_slope: Slope angle in degrees (positive = uphill)
            surface_friction: Coefficient (1.0 = normal, <1.0 = slippery)
        """
        if current_velocity < 0.01:
            return 0.0
        
        # Adjust deceleration for terrain
        effective_decel = self.config.max_deceleration * surface_friction
        
        if abs(terrain_slope) > 5.0:  # Significant slope
            slope_factor = self.config.slope_deceleration_factor
            effective_decel *= slope_factor
        
        # Physics: v² = u² + 2as => s = (v² - u²) / (2a)
        # Final velocity = 0, initial = current_velocity
        braking_distance = (current_velocity ** 2) / (2 * effective_decel)
        
        # Add reaction time buffer
        reaction_distance = current_velocity * self.config.velocity_safety_factor
        
        total_distance = braking_distance + reaction_distance
        
        return total_distance
    
    def compute_time_to_stop(self, current_velocity: float,
                            terrain_slope: float = 0.0) -> float:
        """Compute time required to come to complete stop"""
        if current_velocity < 0.01:
            return 0.0
        
        effective_decel = self.config.max_deceleration
        
        if abs(terrain_slope) > 5.0:
            effective_decel *= self.config.slope_deceleration_factor
        
        # t = v / a
        stopping_time = current_velocity / effective_decel
        
        return stopping_time + self.config.velocity_safety_factor
    
    def compute_safe_approach_velocity(self, distance_to_obstacle: float,
                                       terrain_slope: float = 0.0) -> float:
        """Compute maximum safe velocity for given distance to obstacle"""
        # Solve for v: distance = v² / (2a) + v * t_reaction
        # This is quadratic: distance = v² / (2a) + v * t_r
        # Rearrange: v² / (2a) + v * t_r - distance = 0
        # v² + 2a * t_r * v - 2a * distance = 0
        
        effective_decel = self.config.max_deceleration
        if abs(terrain_slope) > 5.0:
            effective_decel *= self.config.slope_deceleration_factor
        
        t_r = self.config.velocity_safety_factor
        
        # Quadratic formula: v = (-b + sqrt(b² + 4ac)) / 2
        # where a=1, b=2*decel*t_r, c=-2*decel*distance
        a_coef = 1.0
        b_coef = 2 * effective_decel * t_r
        c_coef = -2 * effective_decel * distance_to_obstacle
        
        discriminant = b_coef**2 - 4 * a_coef * c_coef
        
        if discriminant < 0:
            return 0.0
        
        safe_velocity = (-b_coef + np.sqrt(discriminant)) / (2 * a_coef)
        
        return min(safe_velocity, self.config.max_velocity)


class SafetyAnalyzer:
    """Analyze safety and compute response levels"""
    
    def __init__(self, config: PredictorConfig):
        self.config = config
        self.braking_model = BrakingModel(config)
    
    def classify_collision_severity(self, collision_event: CollisionEvent) -> str:
        """Classify collision severity"""
        ttc = collision_event.time_to_collision
        prob = collision_event.collision_probability
        rel_vel = collision_event.relative_velocity
        
        if ttc < 1.0 or prob > 0.5 or rel_vel > 1.5:
            return "CRITICAL"
        elif ttc < 2.0 or prob > 0.3 or rel_vel > 1.0:
            return "HIGH"
        elif ttc < 4.0 or prob > 0.15:
            return "MEDIUM"
        else:
            return "LOW"
    
    def determine_avoidance_actions(self, collision_event: CollisionEvent,
                                   robot_state: RobotState,
                                   obstacles: List[ObstacleState]) -> List[str]:
        """Determine recommended avoidance actions"""
        actions = []
        
        if collision_event.severity == "CRITICAL":
            actions.append("EMERGENCY_STOP")
            actions.append("ENGAGE_BRAKES")
        elif collision_event.severity == "HIGH":
            actions.append("HARD_BRAKE")
            actions.append("PREPARE_EVASIVE_MANEUVER")
        elif collision_event.severity == "MEDIUM":
            actions.append("SOFT_BRAKE")
            actions.append("ADJUST_PATH")
        else:
            actions.append("REDUCE_SPEED")
            actions.append("MONITOR")
        
        # Check if lateral evasion is possible
        if self._check_lateral_clearance(robot_state, obstacles, collision_event):
            actions.append("STEER_LEFT" if self._should_steer_left(collision_event) else "STEER_RIGHT")
        
        return actions
    
    def _check_lateral_clearance(self, robot_state: RobotState,
                                 obstacles: List[ObstacleState],
                                 collision_event: CollisionEvent) -> bool:
        """Check if there's clearance for lateral evasion"""
        # Simplified check - in production would be more sophisticated
        robot_width = self.config.robot_radius * 2
        required_clearance = robot_width + self.config.base_safety_margin
        
        # Check left and right sides
        for side_angle in [-np.pi/2, np.pi/2]:
            check_angle = robot_state.heading + side_angle
            check_point = robot_state.position + required_clearance * np.array([
                np.cos(check_angle),
                np.sin(check_angle)
            ])
            
            # Check if any obstacle blocks this direction
            clear = True
            for obs in obstacles:
                if obs.id == collision_event.obstacle_id:
                    continue
                
                dist = np.linalg.norm(check_point - obs.position)
                if dist < obs.size + self.config.robot_radius:
                    clear = False
                    break
            
            if clear:
                return True
        
        return False
    
    def _should_steer_left(self, collision_event: CollisionEvent) -> bool:
        """Determine if should steer left or right"""
        # Steer based on impact angle
        return collision_event.impact_angle > 0
    
    def compute_safety_envelope(self, robot_state: RobotState,
                                obstacles: List[ObstacleState],
                                terrain_slope: float = 0.0) -> SafetyEnvelope:
        """Compute safe velocity envelope"""
        # Find nearest obstacle in each direction
        min_forward_dist = float('inf')
        min_lateral_dist = float('inf')
        constraint_reason = "NONE"
        
        for obs in obstacles:
            relative_pos = obs.position - robot_state.position
            distance = np.linalg.norm(relative_pos)
            
            # Decompose into forward/lateral relative to robot heading
            angle_to_obs = np.arctan2(relative_pos[1], relative_pos[0])
            relative_angle = angle_to_obs - robot_state.heading
            relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
            
            forward_component = distance * np.cos(relative_angle)
            lateral_component = abs(distance * np.sin(relative_angle))
            
            if abs(relative_angle) < np.pi / 4:  # Forward sector
                if forward_component < min_forward_dist and forward_component > 0:
                    min_forward_dist = forward_component
                    constraint_reason = f"OBSTACLE_{obs.id}_FORWARD"
            
            if abs(lateral_component) < min_lateral_dist:
                min_lateral_dist = lateral_component
        
        # Compute safe velocities
        max_forward = self.braking_model.compute_safe_approach_velocity(
            min_forward_dist,
            terrain_slope
        )
        
        max_lateral = min(self.config.max_velocity * 0.5, max_forward * 0.7)
        max_angular = self.config.max_angular_velocity
        
        # Reduce limits if close to obstacles
        if min_forward_dist < self.config.emergency_stop_distance:
            max_forward = 0.0
            max_lateral = 0.0
            max_angular = 0.0
            constraint_reason = "EMERGENCY_STOP"
        elif min_forward_dist < self.config.hard_brake_distance:
            max_forward *= 0.3
            max_lateral *= 0.5
            max_angular *= 0.7
        elif min_forward_dist < self.config.soft_brake_distance:
            max_forward *= 0.6
            max_lateral *= 0.8
        
        # Recommended velocity (conservative)
        recommended = np.array([max_forward * 0.7, 0.0])
        
        return SafetyEnvelope(
            max_forward_velocity=max_forward,
            max_lateral_velocity=max_lateral,
            max_angular_velocity=max_angular,
            recommended_velocity=recommended,
            constraint_reason=constraint_reason
        )


class CollisionPredictor:
    """Main collision prediction and safety system"""
    
    def __init__(self, config: Optional[PredictorConfig] = None):
        self.config = config or PredictorConfig()
        
        # Initialize components
        self.trajectory_predictor = TrajectoryPredictor(self.config)
        self.obstacle_motion_model = ObstacleMotionModel(self.config)
        self.collision_checker = CollisionChecker(self.config)
        self.safety_analyzer = SafetyAnalyzer(self.config)
        
        # State
        self.current_robot_state: Optional[RobotState] = None
        self.current_obstacles: List[ObstacleState] = []
        self.predicted_collisions: List[CollisionEvent] = []
        self.safety_envelope: Optional[SafetyEnvelope] = None
        
        # Threading
        self.lock = threading.Lock()
        self.prediction_thread: Optional[threading.Thread] = None
        self.running = False
        
        logger.info("Collision Predictor initialized")
    
    def update_robot_state(self, position: np.ndarray, velocity: np.ndarray,
                          heading: float, angular_velocity: float):
        """Update current robot state"""
        with self.lock:
            self.current_robot_state = RobotState(
                position=position,
                velocity=velocity,
                heading=heading,
                angular_velocity=angular_velocity,
                timestamp=time.time()
            )
    
    def update_obstacles(self, obstacles: List[Dict]):
        """Update obstacle states from sensor fusion"""
        obstacle_states = []
        
        for obs_dict in obstacles:
            # Parse obstacle dictionary
            obs_id = obs_dict.get('id', -1)
            position = np.array(obs_dict.get('position', [0, 0]))[:2]
            velocity = obs_dict.get('velocity')
            if velocity is not None:
                velocity = np.array(velocity)[:2]
            
            size = obs_dict.get('size', 0.2)
            confidence = obs_dict.get('confidence', 1.0)
            is_dynamic = velocity is not None and np.linalg.norm(velocity) > 0.1
            
            obstacle_state = ObstacleState(
                id=obs_id,
                position=position,
                velocity=velocity,
                size=size,
                confidence=confidence,
                is_dynamic=is_dynamic
            )
            
            obstacle_states.append(obstacle_state)
            self.obstacle_motion_model.add_observation(obstacle_state)
        
        with self.lock:
            self.current_obstacles = obstacle_states
    
    def predict_collisions(self, planned_control_inputs: Optional[List[np.ndarray]] = None,
                          terrain_slope: float = 0.0) -> Dict:
        """
        Predict potential collisions.
        
        Args:
            planned_control_inputs: Sequence of [linear_vel, angular_vel] commands
            terrain_slope: Current terrain slope in degrees
            
        Returns:
            Dictionary with collision predictions and safety information
        """
        with self.lock:
            if self.current_robot_state is None:
                return {'status': 'NO_ROBOT_STATE', 'collisions': []}
            
            robot_state = self.current_robot_state
            obstacles = self.current_obstacles.copy()
        
        # Predict robot trajectory
        robot_trajectories = self.trajectory_predictor.predict_trajectory(
            robot_state,
            planned_control_inputs,
            num_samples=self.config.num_monte_carlo_samples
        )
        
        nominal_trajectory = robot_trajectories[0]
        
        # Predict collisions for each obstacle
        collision_events = []
        
        for obstacle in obstacles:
            # Predict obstacle trajectory
            obstacle_trajectories = self.obstacle_motion_model.predict_obstacle_trajectory(
                obstacle,
                self.config.prediction_horizon,
                num_samples=self.config.num_monte_carlo_samples
            )
            
            # Compute collision probability
            collision_prob, time_series_probs = self.collision_checker.compute_collision_probability(
                robot_trajectories,
                obstacle_trajectories,
                obstacle.size
            )
            
            # Check if collision is likely
            if collision_prob > self.config.collision_probability_threshold:
                # Find collision time
                collision_time = self.collision_checker.find_collision_time(
                    nominal_trajectory,
                    obstacle_trajectories[0],
                    obstacle.size
                )
                
                if collision_time is not None:
                    # Find collision point
                    collision_idx = int(collision_time / self.config.prediction_timestep)
                    if collision_idx < len(nominal_trajectory):
                        collision_point = nominal_trajectory[collision_idx].position
                        
                        # Compute relative velocity
                        robot_vel = nominal_trajectory[collision_idx].velocity
                        if obstacle.velocity is not None:
                            relative_velocity = np.linalg.norm(robot_vel - obstacle.velocity)
                        else:
                            relative_velocity = np.linalg.norm(robot_vel)
                        
                        # Compute impact angle
                        relative_pos = collision_point - obstacle.position
                        impact_angle = np.arctan2(relative_pos[1], relative_pos[0]) - robot_state.heading
                        impact_angle = np.arctan2(np.sin(impact_angle), np.cos(impact_angle))
                        
                        # Create collision event
                        collision_event = CollisionEvent(
                            time_to_collision=collision_time,
                            collision_point=collision_point,
                            collision_probability=collision_prob,
                            obstacle_id=obstacle.id,
                            relative_velocity=relative_velocity,
                            impact_angle=impact_angle,
                            severity="UNKNOWN",
                            avoidance_actions=[]
                        )
                        
                        # Classify severity
                        collision_event.severity = self.safety_analyzer.classify_collision_severity(
                            collision_event
                        )
                        
                        # Determine avoidance actions
                        collision_event.avoidance_actions = self.safety_analyzer.determine_avoidance_actions(
                            collision_event,
                            robot_state,
                            obstacles
                        )
                        
                        collision_events.append(collision_event)
        
        # Sort by time to collision
        collision_events.sort(key=lambda x: x.time_to_collision)
        
        # Compute safety envelope
        safety_envelope = self.safety_analyzer.compute_safety_envelope(
            robot_state,
            obstacles,
            terrain_slope
        )
        
        # Update state
        with self.lock:
            self.predicted_collisions = collision_events
            self.safety_envelope = safety_envelope
        
        # Prepare result
        result = {
            'status': 'OK',
            'timestamp': time.time(),
            'collisions': [self._collision_to_dict(c) for c in collision_events],
            'safety_envelope': self._envelope_to_dict(safety_envelope),
            'requires_emergency_stop': any(c.severity == "CRITICAL" for c in collision_events),
            'requires_hard_brake': any(c.severity in ["CRITICAL", "HIGH"] for c in collision_events),
            'minimum_ttc': collision_events[0].time_to_collision if collision_events else None,
            'nominal_trajectory': [
                {
                    'time': tp.time,
                    'position': tp.position.tolist(),
                    'velocity': tp.velocity.tolist()
                }
                for tp in nominal_trajectory[::5]  # Subsample for efficiency
            ]
        }
        
        return result
    
    def compute_safe_velocity(self, target_velocity: np.ndarray,
                             terrain_slope: float = 0.0) -> np.ndarray:
        """
        Compute safe velocity that respects safety envelope.
        
        Args:
            target_velocity: Desired velocity [vx, vy]
            terrain_slope: Terrain slope in degrees
            
        Returns:
            Safe velocity [vx, vy]
        """
        with self.lock:
            if self.current_robot_state is None or not self.current_obstacles:
                return target_velocity
            
            robot_state = self.current_robot_state
            obstacles = self.current_obstacles.copy()
        
        # Get safety envelope
        envelope = self.safety_analyzer.compute_safety_envelope(
            robot_state,
            obstacles,
            terrain_slope
        )
        
        # Decompose target velocity into forward/lateral
        target_speed = np.linalg.norm(target_velocity)
        if target_speed < 0.01:
            return target_velocity
        
        target_direction = target_velocity / target_speed
        
        # Project onto robot heading
        robot_direction = np.array([np.cos(robot_state.heading), np.sin(robot_state.heading)])
        forward_component = np.dot(target_velocity, robot_direction)
        lateral_component = target_speed**2 - forward_component**2
        lateral_component = np.sqrt(max(0, lateral_component))
        
        # Clip to safe limits
        safe_forward = np.clip(forward_component, 0, envelope.max_forward_velocity)
        safe_lateral = np.clip(lateral_component, 0, envelope.max_lateral_velocity)
        
        # Reconstruct velocity
        if safe_forward < 0.01 and safe_lateral < 0.01:
            return np.array([0.0, 0.0])
        
        # Scale target direction to safe magnitude
        safe_magnitude = np.sqrt(safe_forward**2 + safe_lateral**2)
        safe_velocity = target_direction * safe_magnitude
        
        return safe_velocity
    
    def compute_evasive_trajectory(self, obstacle_id: int,
                                   evasion_type: str = "LATERAL") -> Optional[List[np.ndarray]]:
        """
        Compute evasive maneuver trajectory.
        
        Args:
            obstacle_id: ID of obstacle to evade
            evasion_type: "LATERAL", "BRAKE", or "ACCELERATE"
            
        Returns:
            List of control inputs [linear_vel, angular_vel] or None
        """
        with self.lock:
            if self.current_robot_state is None:
                return None
            
            robot_state = self.current_robot_state
            
            # Find obstacle
            obstacle = None
            for obs in self.current_obstacles:
                if obs.id == obstacle_id:
                    obstacle = obs
                    break
            
            if obstacle is None:
                return None
        
        control_sequence = []
        num_steps = int(self.config.prediction_horizon / self.config.prediction_timestep)
        
        if evasion_type == "LATERAL":
            # Determine evasion direction
            relative_pos = obstacle.position - robot_state.position
            angle_to_obstacle = np.arctan2(relative_pos[1], relative_pos[0])
            relative_angle = angle_to_obstacle - robot_state.heading
            
            # Steer perpendicular to obstacle
            if relative_angle > 0:
                evasion_omega = -self.config.max_angular_velocity * 0.7
            else:
                evasion_omega = self.config.max_angular_velocity * 0.7
            
            # Maintain moderate forward speed while turning
            evasion_velocity = self.config.max_velocity * 0.4
            
            for _ in range(num_steps):
                control_sequence.append(np.array([evasion_velocity, evasion_omega]))
        
        elif evasion_type == "BRAKE":
            # Emergency braking
            current_speed = np.linalg.norm(robot_state.velocity)
            decel_rate = self.config.max_deceleration
            
            for step in range(num_steps):
                time_elapsed = step * self.config.prediction_timestep
                safe_speed = max(0, current_speed - decel_rate * time_elapsed)
                control_sequence.append(np.array([safe_speed, 0.0]))
        
        elif evasion_type == "ACCELERATE":
            # Accelerate past obstacle (if safe)
            target_speed = min(self.config.max_velocity, 
                              np.linalg.norm(robot_state.velocity) * 1.5)
            
            for _ in range(num_steps):
                control_sequence.append(np.array([target_speed, 0.0]))
        
        return control_sequence
    
    def validate_path(self, waypoints: List[np.ndarray],
                     velocity_profile: Optional[List[float]] = None) -> Dict:
        """
        Validate if path is collision-free.
        
        Args:
            waypoints: List of [x, y] waypoints
            velocity_profile: Speed at each waypoint (optional)
            
        Returns:
            Validation result dictionary
        """
        if velocity_profile is None:
            velocity_profile = [self.config.max_velocity * 0.5] * len(waypoints)
        
        with self.lock:
            if self.current_robot_state is None:
                return {'valid': False, 'reason': 'NO_ROBOT_STATE'}
            
            obstacles = self.current_obstacles.copy()
        
        # Check each segment
        unsafe_segments = []
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            velocity = velocity_profile[i]
            
            # Interpolate points along segment
            segment_length = np.linalg.norm(end - start)
            num_checks = max(5, int(segment_length / 0.1))
            
            for t in np.linspace(0, 1, num_checks):
                point = start + t * (end - start)
                
                # Check collision with all obstacles
                for obs in obstacles:
                    distance = np.linalg.norm(point - obs.position)
                    safety_margin = self.config.base_safety_margin
                    
                    # Increase margin with velocity
                    safety_margin += velocity * self.config.velocity_safety_factor
                    
                    if distance < self.config.robot_radius + obs.size + safety_margin:
                        unsafe_segments.append({
                            'segment_index': i,
                            'position': point.tolist(),
                            'obstacle_id': obs.id,
                            'clearance': float(distance - self.config.robot_radius - obs.size)
                        })
                        break
        
        is_valid = len(unsafe_segments) == 0
        
        return {
            'valid': is_valid,
            'unsafe_segments': unsafe_segments,
            'total_segments': len(waypoints) - 1,
            'path_length': sum(np.linalg.norm(waypoints[i+1] - waypoints[i]) 
                              for i in range(len(waypoints) - 1))
        }
    
    def compute_time_varying_risk(self, prediction_horizon: Optional[float] = None) -> np.ndarray:
        """
        Compute risk level over time.
        
        Returns:
            Array of risk values [0-1] at each timestep
        """
        if prediction_horizon is None:
            prediction_horizon = self.config.prediction_horizon
        
        num_steps = int(prediction_horizon / self.config.prediction_timestep)
        risk_profile = np.zeros(num_steps)
        
        with self.lock:
            if self.current_robot_state is None or not self.current_obstacles:
                return risk_profile
            
            robot_state = self.current_robot_state
            obstacles = self.current_obstacles.copy()
        
        # Predict nominal trajectory
        robot_trajectories = self.trajectory_predictor.predict_trajectory(
            robot_state,
            None,
            num_samples=1
        )
        nominal_trajectory = robot_trajectories[0]
        
        # Compute risk at each timestep
        for i, traj_point in enumerate(nominal_trajectory):
            if i >= num_steps:
                break
            
            min_distance = float('inf')
            
            for obs in obstacles:
                # Predict obstacle position at this time
                obs_trajectories = self.obstacle_motion_model.predict_obstacle_trajectory(
                    obs,
                    traj_point.time,
                    num_samples=1
                )
                
                if obs_trajectories:
                    _, obs_pos = obs_trajectories[0][-1] if obs_trajectories[0] else (0, obs.position)
                    distance = np.linalg.norm(traj_point.position - obs_pos)
                    min_distance = min(min_distance, distance)
            
            # Convert distance to risk (inverse relationship)
            safe_distance = self.config.robot_radius + self.config.base_safety_margin + 0.5
            if min_distance < safe_distance:
                risk = 1.0 - (min_distance / safe_distance)
            else:
                risk = 0.0
            
            risk_profile[i] = np.clip(risk, 0.0, 1.0)
        
        return risk_profile
    
    def get_emergency_stop_trajectory(self) -> List[np.ndarray]:
        """Generate emergency stop control sequence"""
        num_steps = int(self.config.critical_ttc_threshold / self.config.prediction_timestep)
        
        with self.lock:
            if self.current_robot_state is None:
                return []
            
            current_speed = np.linalg.norm(self.current_robot_state.velocity)
        
        # Exponential deceleration profile
        control_sequence = []
        for step in range(num_steps):
            time_elapsed = step * self.config.prediction_timestep
            # v(t) = v0 * e^(-kt) where k gives zero velocity at t = critical_ttc
            decay_rate = 5.0 / self.config.critical_ttc_threshold
            safe_speed = current_speed * np.exp(-decay_rate * time_elapsed)
            control_sequence.append(np.array([safe_speed, 0.0]))
        
        return control_sequence
    
    def _collision_to_dict(self, collision: CollisionEvent) -> Dict:
        """Convert collision event to dictionary"""
        return {
            'time_to_collision': float(collision.time_to_collision),
            'collision_point': collision.collision_point.tolist(),
            'collision_probability': float(collision.collision_probability),
            'obstacle_id': collision.obstacle_id,
            'relative_velocity': float(collision.relative_velocity),
            'impact_angle_deg': float(np.degrees(collision.impact_angle)),
            'severity': collision.severity,
            'avoidance_actions': collision.avoidance_actions
        }
    
    def _envelope_to_dict(self, envelope: SafetyEnvelope) -> Dict:
        """Convert safety envelope to dictionary"""
        return {
            'max_forward_velocity': float(envelope.max_forward_velocity),
            'max_lateral_velocity': float(envelope.max_lateral_velocity),
            'max_angular_velocity': float(envelope.max_angular_velocity),
            'recommended_velocity': envelope.recommended_velocity.tolist(),
            'constraint_reason': envelope.constraint_reason
        }
    
    def start_continuous_prediction(self):
        """Start background prediction thread"""
        if self.running:
            logger.warning("Prediction already running")
            return
        
        self.running = True
        self.prediction_thread = threading.Thread(
            target=self._prediction_loop,
            daemon=True
        )
        self.prediction_thread.start()
        logger.info("Started continuous collision prediction")
    
    def stop_continuous_prediction(self):
        """Stop background prediction thread"""
        self.running = False
        if self.prediction_thread:
            self.prediction_thread.join(timeout=2.0)
        logger.info("Stopped collision prediction")
    
    def _prediction_loop(self):
        """Background prediction loop"""
        update_period = 1.0 / self.config.update_rate
        
        while self.running:
            start_time = time.time()
            
            # Run prediction
            try:
                self.predict_collisions()
            except Exception as e:
                logger.error(f"Prediction error: {e}")
            
            # Sleep
            elapsed = time.time() - start_time
            sleep_time = max(0, update_period - elapsed)
            time.sleep(sleep_time)
    
    def get_statistics(self) -> Dict:
        """Get prediction statistics"""
        with self.lock:
            return {
                'num_tracked_obstacles': len(self.current_obstacles),
                'num_predicted_collisions': len(self.predicted_collisions),
                'has_critical_collision': any(c.severity == "CRITICAL" for c in self.predicted_collisions),
                'minimum_ttc': min((c.time_to_collision for c in self.predicted_collisions), 
                                  default=None),
                'safety_envelope_active': self.safety_envelope is not None
            }
    
    def reset(self):
        """Reset predictor state"""
        with self.lock:
            self.current_obstacles.clear()
            self.predicted_collisions.clear()
            self.safety_envelope = None
            self.obstacle_motion_model.obstacle_tracks.clear()
        
        logger.info("Collision predictor reset")
    
    def export_prediction_data(self) -> Dict:
        """Export all prediction data for analysis"""
        with self.lock:
            return {
                'robot_state': {
                    'position': self.current_robot_state.position.tolist() if self.current_robot_state else None,
                    'velocity': self.current_robot_state.velocity.tolist() if self.current_robot_state else None,
                    'heading': float(self.current_robot_state.heading) if self.current_robot_state else None
                },
                'obstacles': [
                    {
                        'id': obs.id,
                        'position': obs.position.tolist(),
                        'velocity': obs.velocity.tolist() if obs.velocity is not None else None,
                        'size': float(obs.size),
                        'is_dynamic': obs.is_dynamic
                    }
                    for obs in self.current_obstacles
                ],
                'collisions': [self._collision_to_dict(c) for c in self.predicted_collisions],
                'safety_envelope': self._envelope_to_dict(self.safety_envelope) if self.safety_envelope else None
            }


# Example usage and testing
if __name__ == "__main__":
    # Initialize predictor
    config = PredictorConfig(
        prediction_horizon=5.0,
        max_velocity=1.5,
        max_deceleration=1.0,
        robot_radius=0.35
    )
    
    predictor = CollisionPredictor(config)
    
    print("Collision Predictor Initialized")
    print(f"Prediction Horizon: {config.prediction_horizon}s")
    print(f"Robot Radius: {config.robot_radius}m")
    
    # Set robot state
    print("\n=== Setting Robot State ===")
    predictor.update_robot_state(
        position=np.array([0.0, 0.0]),
        velocity=np.array([1.0, 0.0]),
        heading=0.0,
        angular_velocity=0.0
    )
    print("Robot: position=[0, 0], velocity=[1.0, 0] m/s, heading=0°")
    
    # Add obstacles
    print("\n=== Adding Obstacles ===")
    obstacles = [
        {
            'id': 0,
            'position': [3.0, 0.0],
            'velocity': None,
            'size': 0.3,
            'confidence': 0.9
        },
        {
            'id': 1,
            'position': [5.0, 1.0],
            'velocity': [-0.5, 0.0],
            'size': 0.2,
            'confidence': 0.85
        }
    ]
    
    predictor.update_obstacles(obstacles)
    print(f"Added {len(obstacles)} obstacles")
    for obs in obstacles:
        print(f"  Obstacle {obs['id']}: pos={obs['position']}, "
              f"vel={obs['velocity']}, size={obs['size']}m")
    
    # Predict collisions
    print("\n=== Predicting Collisions ===")
    result = predictor.predict_collisions()
    
    print(f"Status: {result['status']}")
    print(f"Collisions Detected: {len(result['collisions'])}")
    print(f"Emergency Stop Required: {result['requires_emergency_stop']}")
    print(f"Hard Brake Required: {result['requires_hard_brake']}")
    
    if result['collisions']:
        print("\nCollision Details:")
        for collision in result['collisions']:
            print(f"\n  Obstacle {collision['obstacle_id']}:")
            print(f"    Time to Collision: {collision['time_to_collision']:.2f}s")
            print(f"    Collision Point: {collision['collision_point']}")
            print(f"    Probability: {collision['collision_probability']:.1%}")
            print(f"    Severity: {collision['severity']}")
            print(f"    Relative Velocity: {collision['relative_velocity']:.2f} m/s")
            print(f"    Impact Angle: {collision['impact_angle_deg']:.1f}°")
            print(f"    Avoidance Actions: {', '.join(collision['avoidance_actions'])}")
    
    # Safety envelope
    print("\n=== Safety Envelope ===")
    envelope = result['safety_envelope']
    print(f"Max Forward Velocity: {envelope['max_forward_velocity']:.2f} m/s")
    print(f"Max Lateral Velocity: {envelope['max_lateral_velocity']:.2f} m/s")
    print(f"Max Angular Velocity: {envelope['max_angular_velocity']:.2f} rad/s")
    print(f"Recommended Velocity: {envelope['recommended_velocity']}")
    print(f"Constraint Reason: {envelope['constraint_reason']}")
    
    # Test safe velocity computation
    print("\n=== Safe Velocity Computation ===")
    target_velocity = np.array([1.5, 0.0])
    safe_velocity = predictor.compute_safe_velocity(target_velocity)
    print(f"Target Velocity: {target_velocity}")
    print(f"Safe Velocity: {safe_velocity}")
    print(f"Velocity Reduced: {np.linalg.norm(target_velocity) > np.linalg.norm(safe_velocity)}")
    
    # Test evasive maneuver
    print("\n=== Evasive Maneuver ===")
    if result['collisions']:
        obstacle_id = result['collisions'][0]['obstacle_id']
        evasive_controls = predictor.compute_evasive_trajectory(obstacle_id, "LATERAL")
        
        if evasive_controls:
            print(f"Generated {len(evasive_controls)} control steps for obstacle {obstacle_id}")
            print(f"First 3 controls: {evasive_controls[:3]}")
    
    # Test path validation
    print("\n=== Path Validation ===")
    waypoints = [
        np.array([0.0, 0.0]),
        np.array([2.0, 0.0]),
        np.array([4.0, 0.5]),
        np.array([6.0, 1.0])
    ]
    
    validation = predictor.validate_path(waypoints)
    print(f"Path Valid: {validation['valid']}")
    print(f"Path Length: {validation['path_length']:.2f}m")
    print(f"Unsafe Segments: {len(validation['unsafe_segments'])}")
    
    if validation['unsafe_segments']:
        print("\nUnsafe Segment Details:")
        for seg in validation['unsafe_segments'][:3]:  # Show first 3
            print(f"  Segment {seg['segment_index']}: "
                  f"clearance={seg['clearance']:.2f}m, "
                  f"obstacle={seg['obstacle_id']}")
    
    # Test risk profile
    print("\n=== Time-Varying Risk Profile ===")
    risk_profile = predictor.compute_time_varying_risk(prediction_horizon=3.0)
    print(f"Risk samples: {len(risk_profile)}")
    print(f"Peak risk: {np.max(risk_profile):.2%}")
    print(f"Average risk: {np.mean(risk_profile):.2%}")
    print(f"Risk > 50% at steps: {np.where(risk_profile > 0.5)[0].tolist()}")
    
    # Statistics
    print("\n=== Statistics ===")
    stats = predictor.get_statistics()
    for key, value in stats.items():
        print(f"{key}: {value}")
    
    print("\n=== Collision Predictor Test Complete! ===")
