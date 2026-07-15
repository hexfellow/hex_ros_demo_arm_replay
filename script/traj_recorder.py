#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-07-15
################################################################

import json
import threading
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class TrajRecorder:
    """机械臂轨迹点记录器

    通过后台键盘监听，在控制循环中记录机械臂的关节状态和末端位姿。

    使用方式:
        recorder = TrajRecorder("points.json")
        recorder.start()

        while robot.is_working():
            rate.sleep()
            recorder.check_and_record(robot)
            # ... 原有的控制逻辑 ...

        recorder.stop()
        recorder.save()

    键盘指令:
        r + Enter  → 记录当前点
        s + Enter  → 保存到文件
        c + Enter  → 清空所有记录点
    """

    def __init__(self, output_path=os.path.join(SCRIPT_DIR, "trajectory.json")):
        self._output_path = output_path
        self._points = {}
        self._idx = 1
        self._record_flag = False
        self._save_flag = False
        self._running = False
        self._thread = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self):
        """启动后台键盘监听线程"""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._input_loop, daemon=True)
        self._thread.start()
        print("[Recorder] Started. Press 'r'+Enter to record, 's'+Enter to save.")

    def stop(self):
        """停止后台监听线程"""
        self._running = False
        self._thread = None

    def check_and_record(self, robot):
        """在控制循环中调用，检查是否有按键记录或保存请求"""
        if self._record_flag:
            self._record_flag = False
            self._do_record(robot)
        if self._save_flag:
            self._save_flag = False
            self.save()

    def record(self, robot):
        """立即记录当前机械臂状态"""
        self._do_record(robot)

    def save(self):
        """将记录点写入 JSON 文件

        Args:
            output_path: 输出路径，默认使用初始化时设置的路径
        """
        path = self._output_path
        data = {"point": self._points}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[Recorder] Saved {len(self._points)} points to {path}")

    def clear(self):
        """清空所有记录点"""
        self._points = {}
        self._idx = 1
        print("[Recorder] Cleared all points.")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _input_loop(self):
        """后台线程：持续读取键盘输入"""
        while self._running:
            try:
                line = sys.stdin.readline().strip().lower()
            except (EOFError, OSError):
                break
            if not self._running:
                break
            if line == "r":
                self._record_flag = True
                print("[Recorder] Record triggered!")
            elif line == "s":
                self._save_flag = True
                print("[Recorder] Save triggered!")
            elif line == "c":
                self.clear()

    def _do_record(self, robot):
        """从 robot 获取状态并记录"""
        state = robot.get_arm_state()
        if state is None:
            print("[Recorder] No arm state available, skipping.")
            return

        pose = state.arm_state.pose

        _d_point =2 
        
        self._points[str(self._idx)] = {
            "jnt": [round(float(v), _d_point) for v in state.arm_state.jnt.position],
            "pose": {
                "position": [
                    round(float(pose.position.x), _d_point),
                    round(float(pose.position.y), _d_point),
                    round(float(pose.position.z), _d_point),
                ],
                "orientation": [
                    round(float(pose.orientation.w), _d_point),
                    round(float(pose.orientation.x), _d_point),
                    round(float(pose.orientation.y), _d_point),
                    round(float(pose.orientation.z), _d_point),
                ],
            },
        }
        print(f"[Recorder] Recorded point {self._idx}")
        self._idx += 1
