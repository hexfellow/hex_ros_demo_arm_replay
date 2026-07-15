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
from regex import P

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from traj_util import DataInterface

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import ( # type: ignore
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
)
from hex_util_ros import HexDynUtilY6

from hex_ros2_ws.src.hex_ros_arm_traj_demo.hex_ros_arm_traj_demo.PointLoader import (
    TaskConfigLoader
)

# from TrajectoryController import (
#     TrajectoryController_Base,
#     Point2Point
    
# )

ARM_DOF = 6
GRIP_DOF = 1


class ArmComp:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("arm_comp")

        ### parameters
        self.__rate_param = self.__data_interface.get_rate_param()
        self.__model_param = self.__data_interface.get_model_param()
        self.__comp_param = self.__data_interface.get_comp_param()
        self.__data_interface.logi(f"work rate: {self.__rate_param['ros']} hz")
        self.__data_interface.logi(
            f"teleop rate: {self.__rate_param['teleop']} hz")
        self.__data_interface.logi(f"model urdf: {self.__model_param['urdf']}")
        self.__data_interface.logi(
            f"extra mass: {self.__comp_param['extra_mass']} kg")

        ### dynamics
        self.__gravity = np.asarray(self.__comp_param["gravity"],
                                    dtype=np.float64)
        self.__dyn_util = HexDynUtilY6(
            model_path=self.__model_param["urdf"],
            last_link="link_6",
            pose_end_in_flange=np.asarray(
                self.__model_param["pose_end_in_flange"], dtype=np.float64),
            gravity=self.__gravity,
        )
        
        # ##### comp para
        # # upward force compensating the weight of the extra end-effector mass
        # self.__extra_force = -self.__dyn_util.get_gravity(
        # ) * self.__comp_param["extra_mass"]

        # ### control presets
        # self.__arm_stable_pos = np.asarray(self.__comp_param["arm_stable_pos"],
        #                                    dtype=np.float64)
        # self.__grip_stable_pos = np.asarray(
        #     self.__comp_param["grip_stable_pos"], dtype=np.float64)
        # self.__arm_kp = np.asarray(self.__comp_param["arm_kp"],
        #                            dtype=np.float64)
        # self.__arm_kd = np.asarray(self.__comp_param["arm_kd"],
        #                            dtype=np.float64)
        # self.__grip_kp = np.asarray(self.__comp_param["grip_kp"],
        #                             dtype=np.float64)
        # self.__grip_kd = np.asarray(self.__comp_param["grip_kd"],
        #                             dtype=np.float64)
        # self.__arrive_threshold = self.__comp_param["arrive_threshold"]



        ### threads
        self.__stop_event = threading.Event()
        self.__teleop_thread = threading.Thread(target=self.__teleop_process)
        self.__teleop_dt = 1.0 / max(float(self.__rate_param["teleop"]), 1.0)

    
        ### mod
        self.__init_mode()
    
    
    def __init_mode(self):
        
        self.__task_config = TaskConfigLoader()
        
        
        ## player
        self.__traj_player:Optional[TrajectoryController_Base] = None
        
        
    def __is_running(self):
        return self.__data_interface.ok() and not self.__stop_event.is_set()
    
    ##############################################################
    # Lifecycle
    ##############################################################
    def start(self):
        self.__stop_event.clear()
        self.__teleop_thread.start()
        self.__init_process()

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

    def __build_stable_ctrl(self) -> HexDcRoboManipCtrl:
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode.JNT,
            grav=HexDcBaseVector3(
                x=float(self.__gravity[0]),
                y=float(self.__gravity[1]),
                z=float(self.__gravity[2]),
            ),
            jnt=HexDcBaseJntFull(
                pos=self.__arm_stable_pos.copy(),
                vel=np.zeros(ARM_DOF),
                eff=np.zeros(ARM_DOF),
                kp=self.__arm_kp.copy(),
                kd=self.__arm_kd.copy(),
                lim_vel=np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]),
                lim_acc=np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0]),
            ),
            pose=self.__default_pose(),
        )
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode.JNT,
            jnt=HexDcBaseJntFull(
                pos=self.__grip_stable_pos.copy(),
                vel=np.zeros(GRIP_DOF),
                eff=np.ones(GRIP_DOF),
                kp=self.__grip_kp.copy(),
                kd=self.__grip_kd.copy(),
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
            self.__move_to_stable("init")
        except Exception:
            traceback.print_exc()

    def __exit_process(self):
        try:
            self.__move_to_stable("exit")
        except Exception:
            traceback.print_exc()

    def __work_process(self):
        self.__data_interface.logi("[arm traj]: start play")
        
        self.__traj_player.start_trajectory()
        
        while self.__is_running():
            
            _target_position = self.__traj_player.get_current_target()
            
            ###update/ rate
            self.__data_interface.sleep()


def main():
    arm_comp = ArmComp()
    try:
        arm_comp.start()
        arm_comp.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
