#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-07-15
################################################################

import json
import numpy as np
import os


class TaskConfigLoader:
    """Load recorded trajectory from trajectory.json and provide config for planners."""

    def __init__(self, config_path=None, planner_type="segmented",
                 num_segments=3, segment_duration=0.01):
        """
        Args:
            config_path: Path to trajectory.json. If None, use default path.
            planner_type: "segmented" for SegmentedTrajectoryPlanner,
                          "simple" for TrajectoryPlanner
            num_segments: Number of segments (SegmentedTrajectoryPlanner only)
            segment_duration: Seconds per waypoint transition
        """
        if config_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, '..', '..', 'jsons',
                                       'trajectory.json')

        with open(config_path, 'r') as f:
            data = json.load(f)

        self.info = data['info']
        self._planner_type = planner_type
        self.num_segments = num_segments
        self.segment_duration = segment_duration

        # Parse points in sequential order
        self.raw_points = []
        for i in range(1, self.info['total_points'] + 1):
            self.raw_points.append(data['point'][str(i)])

        self.joint_waypoints = [p['jnt'] for p in self.raw_points]
        self.dt_s = [p['dt_s'] for p in self.raw_points]

    def get_waypoints(self):
        return self.joint_waypoints

    def get_segment_ends(self):
        """Calculate segment end indices for SegmentedTrajectoryPlanner."""
        n = len(self.joint_waypoints)
        seg_size = n // self.num_segments
        ends = [seg_size * (i + 1) for i in range(self.num_segments)]
        ends[-1] = n  # ensure last end == total count
        return ends

    def get_planner_type(self):
        return self._planner_type
