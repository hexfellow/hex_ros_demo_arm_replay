import time
import numpy as np

from abc import ABC, abstractmethod
from enum import Enum

#  j0 + (j1-j0) * (t-t0) / (t1-t0)
DEFAULT_INIT_POS = [0.0, -1.5, 3.0, 0.0, 0.0, 0.0]

DEFAULT_RETURN_HOME_DURATION = 10.0

class TrajectoryControllerBase(ABC):
    """Abstract base for all trajectory controllers."""

    @abstractmethod
    def start_trajectory(self):
        """Start trajectory execution."""
        pass

    @abstractmethod
    def get_target_position(self):
        """Get the target position at the current moment."""
        pass


class TrajectoryPlanner(TrajectoryControllerBase):
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, timestamps, interpolate='linear', loop=True):
        """
        Initialize trajectory planner
        waypoints: List of waypoints
        timestamps: List of timestamps in seconds (relative, ts[0]=0.0)
        interpolate: Interpolation mode — 's_curve', 'linear', or 'hold'
        loop: If True, repeat trajectory cyclically
        """
        self.waypoints = waypoints
        self.timestamps = timestamps
        self.interpolate = interpolate
        self.loop = loop

        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.trajectory_complete = False
        self.start_time = None
        self.last_target_position = None  # Store last commanded position
        
    def start_trajectory(self):
        """Start trajectory execution"""
        if not self.waypoints:
            return False

        self.trajectory_started = True
        self.trajectory_complete = False
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        

    ############################ comput ############################
    
    def _compute_position(self, segment_index, segment_elapsed, seg_duration):
        """Compute target position at the given segment.

        Dispatches to the mode-specific function based on self.interpolate.
        """
        if self.interpolate == 'No':
            return self._get_hold_position(segment_index)

        start_pos = np.array(self.waypoints[segment_index])
        end_pos = np.array(self.waypoints[segment_index + 1])

        if self.interpolate == 's_curve':
            normalized_time = segment_elapsed / seg_duration
            return self._get_s_curve_position(start_pos, end_pos, normalized_time)
        elif self.interpolate == 'linear':
            return self._get_linear_position(start_pos, end_pos, segment_elapsed, seg_duration)

    def _get_hold_position(self, segment_index):
        """Hold at raw waypoint — no interpolation (stepped motion)."""
        return np.array(self.waypoints[segment_index])

    def _get_s_curve_position(self, start_pos, end_pos, normalized_time):
        """S-curve interpolation — smooth accel/decel via 5th-degree polynomial."""
        s = self._smooth_step(normalized_time)
        return start_pos + s * (end_pos - start_pos)

    def _get_linear_position(self, start_pos, end_pos, elapsed, duration):
        """Linear interpolation — constant speed between waypoints.

        Formula: start_pos + (end_pos - start_pos) * elapsed / duration
        """
        return start_pos + (end_pos - start_pos) * elapsed / duration

    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
      
    # ##################  get #########################

    def get_target_position(self):
        """Get the target position at the current moment"""
        if not self.trajectory_started or not self.waypoints:
            return None

        if self.trajectory_complete:
            return self.last_target_position

        current_time = time.time()
        # ## 已完成时间
        trajectory_time = current_time - self.start_time

        # Handle non-loop completion
        if not self.loop and trajectory_time >= self.timestamps[-1]:
            self.trajectory_complete = True
            self.last_target_position = np.array(self.waypoints[-1])
            return self.last_target_position

        # Loop: wrap time into one cycle
        if self.loop:
            trajectory_time = trajectory_time % self.timestamps[-1]

        # 二分查找抓点 -- 边界判断
        segment_index = np.searchsorted(self.timestamps, trajectory_time, side='right') - 1
        segment_index = max(0, min(segment_index, len(self.waypoints) - 2))

        # 下一个时间点的时间
        segment_elapsed = trajectory_time - self.timestamps[segment_index]
        seg_duration = self.timestamps[segment_index + 1] - self.timestamps[segment_index]

        target_position = self._compute_position(segment_index, segment_elapsed, seg_duration)
        self.current_waypoint_index = segment_index
        self.last_target_position = target_position

        return target_position
    

    def get_last_position(self):
        """Get the last commanded position"""
        return self.last_target_position
        
  
    def get_current_segment_info(self):
        """Get information about the current segment"""
        if not self.trajectory_started:
            return None

        current_time = time.time()
        trajectory_time = current_time - self.start_time

        if self.loop:
            trajectory_time = trajectory_time % self.timestamps[-1]

        segment_index = np.searchsorted(self.timestamps, trajectory_time, side='right') - 1
        segment_index = max(0, min(segment_index, len(self.waypoints) - 2))

        segment_elapsed = trajectory_time - self.timestamps[segment_index]
        seg_duration = self.timestamps[segment_index + 1] - self.timestamps[segment_index]
        segment_progress = segment_elapsed / seg_duration

        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'total_elapsed': trajectory_time
        }

class Move2TargetPlanner(TrajectoryControllerBase):
    """Controller for smooth return to home position"""
    
    def __init__(self, start_position, home_position, duration=5):
        """
        Initialize return home controller
        start_position: Starting position (current position when Ctrl+C is pressed)
        home_position: Target home position
        duration: Duration to reach home position (seconds)
        """
        self.start_position = np.array(start_position)
        self.home_position = np.array(home_position)
        self.duration = duration
        self.done = False
        
    def start_trajectory(self):
        self.start_time = time.time()
        
        
    def get_target_position(self):
        """Get the current target position during return home"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if np.allclose(self.start_position,self.home_position):
            self.done = True
        
        if elapsed_time >= self.duration or self.done:
            return self.home_position, True  # Reached home
        
        # if self.done:
        #     return self.home_position, True  # Reached home
        
        # Calculate normalized time [0, 1]
        t = elapsed_time / self.duration
        
        # Use S-curve interpolation for smooth motion
        s = self._smooth_step(t)
        
        # Interpolate between start and home position
        target_position = self.start_position + s * (self.home_position - self.start_position)
        
        return target_position, False  # Not yet reached home
    
    def _smooth_step(self, t):
        """S-curve interpolation function"""
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3
