
import argparse
import time
import traceback
import numpy as np
from hex_util_runtime import ns_now, HexRate
import sys

# If you want to use the local version of the library, you can uncomment the following lines.
PROJECT_PATH = '/home/hexfellow/ttg/hexfellow/hex_driver_robot'
sys.path.insert(1, f'{PROJECT_PATH}')
sys.path.insert(
    1,
    f'{PROJECT_PATH}/hex_driver_robot/tcp_base/generated')

from hex_driver_robot import HexRobotArcherY6, HexRobotArcherY6Params
from hex_util_msg.dataclass import HexDcBaseVector3

from replay_recorder import TrajRecorder
from replay_stream import TrajStream

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
    args = parser.parse_args()
    mode = args.mode
    ctrl_rate = args.ctrl_rate
    samp_rate = args.samp_rate

    # Compute sampling decimation: record once every N loop iterations
    sample_interval = max(1, round(ctrl_rate / samp_rate))

    params = HexRobotArcherY6Params(
        host="172.18.0.50",
        port=8439,
        ctrl_rate=500,
        state_buffer_size=200,
        sens_ts=False,
        grip_type="empty",
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
            recorder = TrajRecorder("../jsons/trajectory.json")
            recorder.start()
            print("[Mode] full trajectory recording")

        if mode == "stream":
            stream = TrajStream(dec=3)
            stream.start(samp_hz=samp_rate)
            print("[Mode] light-weight streaming")

        cnt = 0

        while robot.is_working():
            rate.sleep()
            if mode == "record":
                assert recorder is not None
                try:
                    recorder.check_and_record(robot)
                except Exception as e:
                    print(f"\033[33m[Recorder] check_and_record error: {e}\033[0m")

            robot.set_arm_mit_cmd({
                "ts_ns": ns_now(),
                "jnt_pos": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "jnt_vel": np.zeros(6),
                "mit_tau": np.zeros(6),
                "mit_kp": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "mit_kd": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "grav": HexDcBaseVector3(0.0, 0.0, -9.8),
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
