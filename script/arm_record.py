
import argparse
import time
import traceback
import numpy as np
from hex_util_runtime import ns_now, HexRate
import sys

from hex_driver_robot import HexRobotArcherY6, HexRobotArcherY6Params
from hex_util_msg.dataclass import HexDcBaseVector3

from traj_recorder import TrajRecorder
from traj_stream import TrajStream

def _stamp_to_ns(stamp) -> int:
    """Convert HexDcBaseTime stamp to nanoseconds."""
    return int(stamp.secs * 1_000_000_000 + stamp.nsecs)

def main() -> None:
    parser = argparse.ArgumentParser(description="Arm trajectory recorder / streamer")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["record", "stream"],
        default="record",
        help="'record': full trajectory recording (default), 'stream': light-weight streaming only",
    )
    parser.add_argument(
        "--ctrl-rate",
        type=int,
        default=1000,
        help="Control loop rate in Hz (default: 1000, passed to HexRate)",
    )
    parser.add_argument(
        "--samp-rate",
        type=int,
        default=100,
        help="Sampling rate in Hz (default: 100, controls how often data is recorded)",
    )
    parser.add_argument(
        "--ip",
        type=str,
        required=True,
        help="Robot IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=True,
        help="Robot port",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output file path for trajectory data",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="archer_y6",
        choices=["archer_y6", "firefly_y6"],
        help="Robot type name (e.g. archer_y6, firefly_y6)",
    )
    
    parser.add_argument(
        "--grip-type",
        type=str,
        default="empty",
        choices=["empty", "gp80", "gr100", "gp100"],
        help="Grip type (empty or gp100)",
    )
    
    args = parser.parse_args()
    mode = args.mode
    ctrl_rate = args.ctrl_rate
    samp_rate = args.samp_rate

    # Compute sampling decimation: record once every N loop iterations
    sample_interval = max(1, round(ctrl_rate / samp_rate))

    params = HexRobotArcherY6Params(
        host=args.ip,
        port=args.port,
        ctrl_rate=500,
        state_buffer_size=200,
        sens_ts=False,
        grip_type=args.grip_type,
    )
    robot = None
    recorder = None
    stream = None
    try:
        robot = HexRobotArcherY6(params)
        robot.start()
        print(f"dofs: {robot.get_dofs()}")

        rate = HexRate(ctrl_rate)

        if mode == "record":
            recorder = TrajRecorder(
                args.output,
                robot_type=args.robot_type,
                gripper_type=params.grip_type,
            )
            recorder.start()
            print("[Mode] full trajectory recording")

        if mode == "stream":
            stream = TrajStream(
                dec=3,
                robot_type=args.robot_type,
                gripper_type=params.grip_type,
            )
            stream.start(output_path=args.output, samp_hz=samp_rate)
            print("[Mode] light-weight streaming")

        cnt = 0

        while robot.is_working():
            rate.sleep()
            
            robot.set_arm_mit_cmd({
                "ts_ns": ns_now(),
                "jnt_pos": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "jnt_vel": np.zeros(6),
                "mit_tau": np.zeros(6),
                "mit_kp": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "mit_kd": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "grav": HexDcBaseVector3(0.0, 0.0, -9.8),
            })

            # 夹爪 MIT 阻抗补偿控制，录制期间保持夹爪位置
            robot.set_grip_mit_cmd({
                "ts_ns": ns_now(),
                "jnt_pos": np.array([0.0]),
                "jnt_vel": np.zeros(1),
                "mit_tau": np.zeros(1),
                "mit_kp": np.array([0.0]),
                "mit_kd": np.array([0.0]),
            })

            cnt += 1
            if cnt >= sample_interval:
                cnt = 0
                if mode == "stream":
                    assert stream is not None
                    try:
                        stream.record(robot)
                    except Exception as e:
                        print(f"\033[33m[TrajStream] record error: {e}\033[0m")
            
                if mode == "record":
                    assert recorder is not None
                    try:
                        recorder.check_and_record(robot)
                    except Exception as e:
                        print(f"\033[33m[Recorder] check_and_record error: {e}\033[0m")
            

    except KeyboardInterrupt:
        pass
    except (ConnectionError, ConnectionRefusedError, TimeoutError) as e:
        print(f"\033[31mConnection failed: {e}\033[0m")
    except Exception:
        traceback.print_exc()
    finally:
        if robot is not None:
            robot.stop()
            print("robot stopped cleanly")
        if mode == "record":
            if recorder is not None:
                try:
                    recorder.stop()
                except Exception as e:
                    print(f"\033[33m[Recorder] stop error: {e}\033[0m")
        if mode == "stream":
            try:
                if stream is not None:
                    stream.stop()
            except Exception as e:
                print(f"\033[33m[TrajStream] stop error: {e}\033[0m")


if __name__ == "__main__":
    main()
