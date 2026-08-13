#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 taigong26. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-07-15
################################################################


import os
import sys
import time
import traceback
import threading
from typing import Optional

import numpy as np

# from ament_index_python.packages import get_package_share_directory

from replay_util import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
)

from PointLoader import TaskConfigLoader
from TrajectoryController import (
    TrajectoryPlanner,
    Move2TargetPlanner,
)

ARM_DOF = 6
GRIP_DOF = 1

class ArmReplay:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_replay")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__traj_param = self.__data_interface.get_traj_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"traj rate: {self.__rate_param['traj']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")
        self.__data_interface.logi(f"model urdf: {self.__model_param['urdf']}")

        ### trajectory publish decimation (rate_ros / rate_traj)
        self.__traj_decim = max(
            1,
            int(round(self.__rate_param['ros'] / self.__rate_param['traj'])),
        )

        ### control presets for JNT mode commands
        self.__arm_lim_vel = np.asarray(
            self.__traj_param["lim_vel"], dtype=np.float64)
        self.__arm_lim_acc = np.asarray(
            self.__traj_param["lim_acc"], dtype=np.float64)
        self.__arm_jnt_eff = np.asarray(
            self.__traj_param["jnt_eff"], dtype=np.float64)
        self.__arrive_threshold = 0.1
        
        
        self.__grip_stable_pos = np.zeros(GRIP_DOF, dtype=np.float64)

        ### threads
        self.__stop_event = threading.Event()
        self.__start_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)
        
        ### mod
        self.__init_mode()
    
    def __init_mode(self):
        try:
            # Read waypoints_path from parameters
            waypoints_path = self.__traj_param.get("waypoints_path", "")

            # Determine config path
            if waypoints_path:
                self.__data_interface.logi(f"[init mode]: get path : {waypoints_path}")
            else:
                self.__data_interface.loge(f"[init mode]: Dont get waypoints path ")
                sys.exit(1)

            config_loader = TaskConfigLoader(config_path=waypoints_path)
            waypoints = config_loader.get_waypoints()
            ts_list = config_loader.get_timestamps()

            # Create the arm trajectory player
            self.__traj_player: Optional[TrajectoryPlanner] = \
                TrajectoryPlanner(
                    waypoints=waypoints,
                    timestamps=ts_list,
                )
            self.__data_interface.logi(
                f"TrajectoryPlanner, "
                f"{len(waypoints)} waypoints, "
                f"duration={ts_list[-1]:.3f}s, "
                f"interpolate=Linear")

            # Create the grip trajectory player if gripper data available
            self.__grip_player: Optional[TrajectoryPlanner] = None
            if config_loader.has_grip():
                grip_waypoints = config_loader.get_grip_position()
                self.__grip_player = TrajectoryPlanner(
                    waypoints=grip_waypoints,
                    timestamps=ts_list,
                )
                self.__data_interface.logi(
                    f"GripTrajectoryPlanner, "
                    f"{len(grip_waypoints)} waypoints, "
                    f"gripper_type={config_loader.gripper_type}")
            else:
                self.__data_interface.logi(
                    f"GripTrajectoryPlanner disabled "
                    f"(gripper_type={config_loader.gripper_type})")

        except Exception as e:
            traceback.print_exc()
            self.__data_interface.loge(f"init mod err,  {e} \n")
            sys.exit(1)
            
    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()
    
    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__start_event.clear()
        self.__teleop_thread.start()
        self.__init_process()
        self.__data_interface.logi("start work")

    def run(self):
        try:
            self.__work_process()
        except KeyboardInterrupt:
            pass
        except Exception:
            traceback.print_exc()
        finally:
            self.stop()

    def stop(self):
        self.__stop_event.set()
        if self.__teleop_thread.is_alive():
            self.__teleop_thread.join()
        self.__exit_process()
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass

    ##############################################################
    # Control builders
    ##############################################################
    @staticmethod
    def __default_pose() -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            orientation=HexDcBaseQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

    def __build_traj_ctrl(self, target_pos: np.ndarray) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            jnt=HexDcBaseJntFull(
                pos=target_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=self.__arm_jnt_eff.copy(),
                kp=np.zeros(ARM_DOF),
                kd=np.zeros(ARM_DOF),
                lim_vel=self.__arm_lim_vel.copy(),
                lim_acc=self.__arm_lim_acc.copy(),
            ),
            pose=self.__default_pose(),
        )

        # Grip position: use trajectory interpolation if available
        if self.__grip_player is not None:
            grip_pos = self.__grip_player.get_target_position()
            if grip_pos is not None:
                grip_pos_val = grip_pos
            else:
                grip_pos_val = self.__grip_stable_pos
        else:
            grip_pos_val = self.__grip_stable_pos

        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=grip_pos_val.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.ones(GRIP_DOF),
                kp=np.zeros(GRIP_DOF),
                kd=np.zeros(GRIP_DOF),
                lim_vel=np.array([0.5]),
                lim_acc=np.array([1.0]),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

    ##############################################################
    # Smooth motion helpers
    ##############################################################
    def __move_first_target(self):
        """move to waypoints[0]"""
        if self.__traj_player is None:
            self.__data_interface.logw("no waypoints, skip move_first_target")
            return
        
        waypoint0 = np.asarray(self.__traj_player.get_first_point(), dtype=np.float64)
        self.__data_interface.logi(f"moving to waypoint[0]: {waypoint0}")

        state = None
        _timeout = time.time() + 5
        while state is None:
            try:
                state = self.__data_interface.get_manip_state(latest=True)
            except Exception:
                state = None
                
            self.__data_interface.sleep()
            if time.time() > _timeout:
                self.__data_interface.loge("no manip state, skip move_first_target")
                return
        
        current_pos = np.asarray(
            state.manip_state.arm_state.jnt.position, dtype=np.float64)

        if np.allclose(current_pos, waypoint0, atol=self.__arrive_threshold):
            self.__data_interface.logi("already at waypoint[0]")
            return

        planner = Move2TargetPlanner(
            start_position=current_pos,
            home_position=waypoint0,
            duration=self.__traj_param["expected_time"],
        )
        planner.start_trajectory()

        traj_count = 0
        while self.__data_interface.ok():
            traj_count += 1
            if traj_count >= self.__traj_decim:
                traj_count = 0
                target_pos, done = planner.get_target_position()
                ctrl = self.__build_traj_ctrl(target_pos)
                self.__data_interface.pub_manip_ctrl(ctrl)
                if done:
                    break
            self.__data_interface.sleep()

        self.__data_interface.logi("reached waypoint[0]")

    def __return_to_home(self):
        """return home"""
        self.__data_interface.logi("returning to home")

        home_pos = np.asarray(self.__traj_param["end_position"], dtype=np.float64)

        start_pos = None
        if self.__traj_player is not None:
            last_pos = self.__traj_player.get_last_position()
            if last_pos is not None:
                start_pos = np.asarray(last_pos, dtype=np.float64)

        if start_pos is None:
            state = self.__data_interface.get_manip_state(latest=True)
            if state is None:
                self.__data_interface.logw("cannot get start pos, skip")
                return
            start_pos = np.asarray(
                state.manip_state.arm_state.jnt.position, dtype=np.float64)

        if np.allclose(start_pos, home_pos, atol=self.__arrive_threshold):
            self.__data_interface.logi("already at home")
            return

        planner = Move2TargetPlanner(
            start_position=start_pos,
            home_position=home_pos,
            duration=self.__traj_param["expected_time"],
        )
        planner.start_trajectory()

        traj_count = 0
        while self.__data_interface.ok():
            traj_count += 1
            if traj_count >= self.__traj_decim:
                traj_count = 0
                target_pos, done = planner.get_target_position()
                ctrl = self.__build_traj_ctrl(target_pos)
                self.__data_interface.pub_manip_ctrl(ctrl)
                if done:
                    self.__data_interface.logi("reached home")
                    break
            self.__data_interface.sleep()

    ##############################################################
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
        prev_s = False
        while self.__is_running():
            time.sleep(self.__teleop_dt)

            keys = self.__data_interface.get_keyboard_state(latest=True)
            if keys is None:
                continue

            curr_q = bool(keys.key_q)
            if curr_q and not prev_q:
                self.__data_interface.logi("stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

            curr_s = bool(keys.key_s)
            if curr_s and not prev_s:
                self.__start_event.set()
            prev_s = curr_s

    def __init_process(self):
        try:
            self.__move_first_target()

        except Exception as e:
            self.__data_interface.loge(f"init process err,  {e} \n")
            # traceback.print_exc()

    def __exit_process(self):
        try:
            self.__return_to_home()
        except Exception as e:
            # traceback.print_exc()
            self.__data_interface.loge(f"init process err,  {e} \n")

    def __work_process(self):
        
        if not self.__is_running():
            return

        self.__data_interface.logi("press 's' to start work...")
        while self.__is_running() and not self.__start_event.is_set():
            self.__data_interface.sleep()

        self.__data_interface.logi("start replay")

        self.__replay_traj()

    def __replay_traj(self):
        if self.__traj_player is None:
            self.__data_interface.loge("no trajectory player")
            return

        if not self.__traj_player.start_trajectory():
            self.__data_interface.loge("failed to start trajectory")
            return

        if self.__grip_player is not None:
            self.__grip_player.start_trajectory()
            self.__data_interface.logi("grip trajectory started")

        _send_exit_msg = False
        
        traj_count = 0
        while self.__is_running():
            try: 
                traj_count += 1
                if traj_count >= self.__traj_decim:
                    traj_count = 0
                    target_pos = self.__traj_player.get_target_position()
                    
                    if target_pos is not None:
                        ctrl = self.__build_traj_ctrl(target_pos)
                        self.__data_interface.pub_manip_ctrl(ctrl)
                        self.__data_interface.logd(f"pos: {target_pos[1]}")
                        
                self.__data_interface.sleep()
                
                
                # ## exit
                _done = self.__traj_player.is_done()
                if _done and not _send_exit_msg:
                    self.__data_interface.logi("Task finished, press 'q' to Return Home...")
                    _send_exit_msg = True
                
            except Exception:
                traceback.print_exc()
        
        
def main():
    arm_replay = ArmReplay()
    try:
        arm_replay.start()
        arm_replay.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
