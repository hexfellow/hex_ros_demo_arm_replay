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

    def __init__(self, config_path=None,
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
        self.num_segments = num_segments
        self.segment_duration = segment_duration

        # Parse points in sequential order
        self.raw_points = []
        for i in range(1, self.info['total_points'] + 1):
            self.raw_points.append(data['point'][str(i)])

        self.joint_waypoints = [p['jnt'] for p in self.raw_points]

        # Compute dt_s from consecutive timestamps (ts_ns is always present)
        self.dt_s = [0.0]
        for idx in range(1, len(self.raw_points)):
            dt = (self.raw_points[idx]['ts_ns'] - self.raw_points[idx - 1]['ts_ns']) * 1e-9
            self.dt_s.append(dt)

    def get_waypoints(self):
        return self.joint_waypoints

    def get_timestamps(self):
        """Return timestamps in seconds, relative to first point (ts[0]=0.0).

        One timestamp per waypoint, preserving the original time spacing.
        """
        base_ns = self.raw_points[0]['ts_ns']
        return [(p['ts_ns'] - base_ns) * 1e-9 for p in self.raw_points]

    def get_segment_ends(self):
        """Calculate segment end indices for SegmentedTrajectoryPlanner."""
        n = len(self.joint_waypoints)
        seg_size = n // self.num_segments
        ends = [seg_size * (i + 1) for i in range(self.num_segments)]
        ends[-1] = n  # ensure last end == total count
        return ends


    def get_time_interval(self) -> float:
        """获取相邻轨迹点之间的平均时间间隔（秒）

        优先从 info.samp_hz 计算；若字段不存在则通过起始/结束时间戳和总点数推算。
        """
        if 'samp_hz' in self.info:
            samp_hz = self.info['samp_hz']
            if samp_hz > 0:

                return 1.0 / samp_hz

        # 降级：从 metadata 推算平均间隔
        total_ns = self.info['end_time_ns'] - self.info['start_time_ns']
        n = self.info['total_points']
        if n <= 1 or total_ns <= 0:
            return 1.0
        return total_ns * 1e-9 / n

    def get_interpolate_type(self) -> str:
        """根据是否有采样频率字段决定插值类型

        Returns:
            'No'   — samp_hz 存在，规整采样，不插值（保持原始点）
            'linear' — 字段缺失，不规律录制，线性插值平滑过渡
        """
        return 'No' if 'samp_hz' in self.info else 'linear'
