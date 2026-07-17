
import json
import os

# 脚本所在目录的绝对路径，作为输出目录的基准
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class TrajStream:
    """连续轨迹记录器

    逐帧追加写入文件，不占用内存，支持长时间连续录制。
    控制频率由外部控制循环决定。

    使用方式:
        stream = TrajStream()
        stream.start(samp_hz=100)

        while robot.is_working():
            rate.sleep()
            stream.record(robot)

        stream.stop()
    """

    def __init__(self, dec=2):
        """
        Args:
            dec: 小数精度
        """
        self._dec = dec
        self._f = None
        self._seq = 0
        self._last_abs_ns = None
        self._rel_ns = 0

    # ------------------------------------------------------------------
    # tools
    # ------------------------------------------------------------------

    @staticmethod
    def _ts_to_ns(stamp) -> int:
        """将 HexDcBaseTime 转换为纳秒"""
        return int(stamp.secs * 1_000_000_000 + stamp.nsecs)

    @staticmethod
    def _ffmt(val: int, width: int) -> bytes:
        """固定宽度格式化，用于原地更新 info 头部"""
        return str(val).rjust(width).encode()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, output_path=os.path.join(SCRIPT_DIR, "trajectory.json"), samp_hz=None):
        """开始记录（新建文件 + 写 JSON 头部）

        Args:
            output_path: 输出文件路径
            samp_hz: 采样时间，由外部控制循环决定，写入 JSON metadata
        """
        self._seq = 0
        self._last_abs_ns = None
        self._rel_ns = 0
        self._output_path = output_path

        self._f = open(output_path, "wb")
        self._f.write(b'{\n')
        self._f.write(b'  "info": {\n')
        self._f.write(b'    "start_time_ns": ')
        self._info_anchor_start = self._f.tell()
        self._f.write(self._ffmt(0, 22))
        self._f.write(b',\n')
        self._f.write(b'    "end_time_ns":   ')
        self._info_anchor_end = self._f.tell()
        self._f.write(self._ffmt(0, 22))
        self._f.write(b',\n')
        self._f.write(b'    "total_points":  ')
        self._info_anchor_total = self._f.tell()
        self._f.write(self._ffmt(0, 12))
        self._f.write(b',\n')
        if samp_hz is not None:
            self._f.write(f'    "samp_hz": {samp_hz},\n'.encode())
        self._f.write(b'    "dof": 6\n')
        self._f.write(b'  },\n')
        self._f.write(b'  "point": {\n')
        self._f.flush()
        print(f"[TrajStream] Recording to {output_path}")

    def record(self, robot):
        """记录当前帧的机械臂状态，追加到文件

        Args:
            robot: HexRobotArcherY6 实例
        """
        state = robot.get_arm_state()
        if state is None:
            return

        ts_ns = self._ts_to_ns(state.header.stamp)
        dec = self._dec

        # 纯计算，不碰状态
        if self._last_abs_ns is None:
            new_rel_ns = 0  # 首点 = 0
        else:
            new_rel_ns = self._rel_ns + (ts_ns - self._last_abs_ns)  # 累加时间差
        idx = self._seq + 1  # 1-based 序号

        point = {
            "ts_ns": new_rel_ns,
            "jnt": [round(float(v), dec) for v in state.arm_state.jnt.position],
        }

        # I/O —— 失败则 return
        try:
            line = json.dumps(point, ensure_ascii=False)
            self._f.write(f'    "{idx}": {line},\n'.encode())
            self._f.flush()
        except OSError as e:
            print(f"\033[33m[TrajStream] Write error: {e}\033[0m")
            return

        # I/O 成功后才 commit 状态
        self._last_abs_ns = ts_ns
        self._rel_ns = new_rel_ns
        self._seq += 1

    def stop(self):
        """结束记录：去掉末尾逗号、闭合 JSON、修正 info 头部"""
        if self._f is None:
            return self._output_path
        fp = self._f

        # 1. 去掉最后一个逗号，闭合 JSON
        try:
            fp.seek(-2, os.SEEK_END)      # 回退到 ",\n"
            fp.truncate()
            fp.write(b'\n  }\n}\n')
        except OSError as e:
            print(f"\033[33m[TrajStream] stop truncate/write error: {e}\033[0m")
        try:
            fp.close()
        except OSError as e:
            print(f"\033[33m[TrajStream] stop close error: {e}\033[0m")
        self._f = None

        # 2. 原地修正 info 头部中的元数据（固定宽度覆盖）
        try:
            with open(self._output_path, "r+b") as f:
                f.seek(self._info_anchor_start)
                f.write(self._ffmt(0, 22))
                f.seek(self._info_anchor_end)
                f.write(self._ffmt(self._rel_ns if self._rel_ns else 0, 22))
                f.seek(self._info_anchor_total)
                f.write(self._ffmt(self._seq if self._seq else 0, 12))
        except OSError as e:
            print(f"\033[33m[TrajStream] stop metadata error: {e}\033[0m")

        print(f"[TrajStream] Done: {self._seq} points -> {self._output_path}")
        return self._output_path
