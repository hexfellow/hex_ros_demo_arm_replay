
import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class TrajStream:
    def __init__(self, dec=2):

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
        return int(stamp.secs * 1_000_000_000 + stamp.nsecs)

    @staticmethod
    def _ffmt(val: int, width: int) -> bytes:
        return str(val).rjust(width).encode()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, output_path=os.path.join(SCRIPT_DIR, "trajectory.json"), samp_hz=None):

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

        state = robot.get_arm_state()
        if state is None:
            return

        ts_ns = self._ts_to_ns(state.header.stamp)
        dec = self._dec

        if self._last_abs_ns is None:
            new_rel_ns = 0  # 首点 = 0
        else:
            new_rel_ns = self._rel_ns + (ts_ns - self._last_abs_ns)  # 累加时间差
        idx = self._seq + 1  # 1-based 序号

        point = {
            "ts_ns": new_rel_ns,
            "jnt": [round(float(v), dec) for v in state.arm_state.jnt.position],
        }

        try:
            line = json.dumps(point, ensure_ascii=False)
            self._f.write(f'    "{idx}": {line},\n'.encode())
            self._f.flush()
        except OSError as e:
            print(f"\033[33m[TrajStream] Write error: {e}\033[0m")
            return

        self._last_abs_ns = ts_ns
        self._rel_ns = new_rel_ns
        self._seq += 1

    def stop(self):
        if self._f is None:
            return self._output_path
        fp = self._f

        try:
            fp.seek(-2, os.SEEK_END)     
            fp.truncate()
            fp.write(b'\n  }\n}\n')
        except OSError as e:
            print(f"\033[33m[TrajStream] stop truncate/write error: {e}\033[0m")
        try:
            fp.close()
        except OSError as e:
            print(f"\033[33m[TrajStream] stop close error: {e}\033[0m")
        self._f = None

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
