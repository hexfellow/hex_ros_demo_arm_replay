#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

import os
import sys
import time
import traceback
import threading
from typing import Optional

import numpy as np
import pexpect

from ament_index_python.packages import get_package_share_directory

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from traj_util import DataInterface

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
from .TrajectoryController import (
    TrajectoryControllerBase,
    TrajectoryPlanner,
)

ARM_DOF = 6
GRIP_DOF = 1


class ArmComp:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_traj")

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
        self.__arm_stable_pos = np.asarray(
            self.__traj_param["init_position"], dtype=np.float64)
        self.__grip_stable_pos = np.zeros(GRIP_DOF, dtype=np.float64)
        self.__arm_lim_vel = np.asarray(
            self.__traj_param["lim_vel"], dtype=np.float64)
        self.__arm_lim_acc = np.asarray(
            self.__traj_param["lim_acc"], dtype=np.float64)
        self.__arm_jnt_eff = np.asarray(
            self.__traj_param["jnt_eff"], dtype=np.float64)
        self.__arrive_threshold = 0.1

        ### threads
        self.__stop_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)
        
        ### mod
        self.__init_mode()
    
    
    def __init_mode(self):
        try:
            
            # Load waypoints from JSON
            pkg_share = get_package_share_directory('hex_ros_arm_traj_demo')
            config_path = os.path.join(pkg_share, 'jsons', 'trajectory2.json')
            
            self.__data_interface.logd(f"[init mode]: get path : {config_path}")
            
            config_loader = TaskConfigLoader(config_path=config_path)
            waypoints = config_loader.get_waypoints()
            ts_list = config_loader.get_timestamps()
            interpolate = config_loader.get_interpolate_type()

            self.__data_interface.logd(f"[init mode]: get path : {config_path}")

            init_pos = self.__arm_stable_pos.copy()

            # Create the trajectory player

            self.__traj_player: Optional[TrajectoryControllerBase] = \
                TrajectoryPlanner(
                    waypoints=waypoints,
                    timestamps=ts_list,
                    interpolate=interpolate,
                )
            self.__data_interface.logi(
                f"[arm_traj]: TrajectoryPlanner, "
                f"{len(waypoints)} waypoints, "
                f"duration={ts_list[-1]:.3f}s, "
                f"interpolate={interpolate}")


        except:
            traceback.print_exc()
    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()
    
    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__teleop_thread.start()
        # self.__init_process()

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
        # self.__exit_process()
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

    def __build_stable_ctrl(self) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            jnt=HexDcBaseJntFull(
                pos=self.__arm_stable_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=self.__arm_jnt_eff.copy(),
                kp=np.zeros(ARM_DOF),
                kd=np.zeros(ARM_DOF),
                lim_vel=self.__arm_lim_vel.copy(),
                lim_acc=self.__arm_lim_acc.copy(),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=self.__grip_stable_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.ones(GRIP_DOF),
                kp=np.zeros(GRIP_DOF),
                kd=np.zeros(GRIP_DOF),
                lim_vel=np.array([0.5]),
                lim_acc=np.array([1.0]),
            ),
        )
        return HexDcRoboManipCtrl(arm_ctrl=arm_ctrl, grip_ctrl=grip_ctrl)

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
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=self.__grip_stable_pos.copy(),
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
    # Processes
    ##############################################################
    def __teleop_process(self):
        prev_q = False
        while self.__is_running():
            time.sleep(self.__teleop_dt)

            keys = self.__data_interface.get_keyboard_state(latest=True)
            if keys is None:
                continue

            curr_q = bool(keys.key_q)
            if curr_q and not prev_q:
                self.__data_interface.logi("[arm_comp]: stop and exit")
                self.__stop_event.set()
            prev_q = curr_q

    def __move_to_stable(self, phase: str):
        self.__data_interface.logi(f"[arm_comp]: move to {phase} position")
        stable_ctrl = self.__build_stable_ctrl()
        while self.__data_interface.ok():
            state = self.__data_interface.get_manip_state(latest=True)
            if state is not None:
                jnt_pos = np.asarray(
                    state.manip_state.arm_state.jnt.position,
                    dtype=np.float64,
                )
                if jnt_pos.shape == self.__arm_stable_pos.shape:
                    err = self.__arm_stable_pos - jnt_pos
                    if np.fabs(err).max() < self.__arrive_threshold:
                        break
                self.__data_interface.pub_manip_ctrl(stable_ctrl)
            self.__data_interface.sleep()

    def __init_process(self):
        try:
            ### TODO： 缓慢上线到一个target，默认是waypoint的第一个点。
            self.__move_to_stable("init")
            ### TODO： wait order
        except Exception:
            traceback.print_exc()

    def __exit_process(self):
        try:
            ### TODO： 停止后先不推出，等待指令缓慢上线到下一个点。
            self.__move_to_stable("exit")
            ### TODO： wait order
        except Exception:
            traceback.print_exc()

    def __work_process(self):
        self.__data_interface.logi("[arm traj]: start play")

        self.__data_interface.logd(f"work start")

        if self.__traj_player is None:
            self.__data_interface.loge("[arm traj]: no trajectory player")
            return


        if not self.__traj_player.start_trajectory():
            self.__data_interface.loge("[arm traj]: failed to start trajectory")
            return

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
            except Exception:
                traceback.print_exc()

def main():
    arm_comp = ArmComp()
    try:
        arm_comp.start()
        arm_comp.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
