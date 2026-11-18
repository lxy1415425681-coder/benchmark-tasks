"""servedemo: closed-loop scripted demos for yam_arena task files (privileged state, IK, both arms at once).

A demo script is a function ``script(ctx)`` that drives the arms with motion primitives and can read the live
object states (``ctx.states()``), so grasps follow wherever the cable actually is. Each arm runs its own
generator; ``ctx.both(a, b)`` runs two in parallel.

    python servedemo.py TASK_FILE OUT_DIR [--max-steps 2400]

TASK_FILE must define ``demo_script(ctx)``. Writes OUT_DIR/{top_cam,left_cam,right_cam}.mp4 (H.264, 20 fps),
OUT_DIR/setup.png (first top-camera frame) and OUT_DIR/result.json (task_score, steps, success).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def grip_rot(yaw: float = 0.0, pitch: float = 90.0) -> np.ndarray:
    """TCP rotation: the straight-down grip (fingers closing along world y) tilted so its approach axis is ``pitch``
    deg below horizontal pointing along +x, then turned ``yaw`` rad about the vertical."""
    import math

    from yam_arena.demo import grip_rotation
    from yam_arena.urdf import axis_angle_matrix

    tilt = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -math.radians(90.0 - pitch))
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ tilt @ grip_rotation(0.0)


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF) from wherever its last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena.constants import FINGER_OPEN, READY

        self.name = name
        self.base = np.asarray(base_pos, dtype=float)
        self.q = np.array(READY, dtype=float)
        self.finger = FINGER_OPEN
        self.cmd = np.r_[self.q, 1.0]
        self._yaw = 0.0
        self._pitch = 90.0

    def tcp(self) -> np.ndarray:
        from yam_arena import ik

        return ik.tcp_pose(self.q, self.base)[0]

    def move(self, pos, *, yaw: float = 0.0, pitch: float = 90.0, finger: float | None = None, duration: float = 1.0,
             label: str = ""):
        """Generator: go to ``pos`` (world m, TCP) in a straight line (smoothstep) with the grip turned ``yaw`` about
        the vertical and its approach axis ``pitch`` degrees below horizontal (90: straight down; 0: level, pointing
        along +x turned by yaw); ``finger`` in m (FINGER_OPEN..FINGER_CLOSED; None keeps it). One yield per step."""
        from yam_arena import ik

        f0 = self.finger
        f = self.finger if finger is None else finger
        p0, r0 = ik.tcp_pose(self.q, self.base)
        p1, r1 = np.asarray(pos, dtype=float), grip_rot(yaw, pitch)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            q, _, _ = ik.solve(p0 + s * (p1 - p0), ik.matrix_to_quat_xyzw(ik.slerp_matrix(r0, r1, s)), self.q,
                               base_pos=self.base, iters=60)
            self.q = q
            fk = f0 + s * (f - f0)
            from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

            grip = float(np.clip(fk / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
            self.cmd = np.r_[q, grip]
            yield
        self.finger = f
        self._yaw, self._pitch = yaw, pitch

    def grip(self, finger: float, duration: float = 0.6):
        """Generator: open/close in place."""
        yield from self.move(self.tcp(), yaw=self._yaw, pitch=self._pitch, finger=finger, duration=duration)

    def wait(self, duration: float):
        for _ in range(max(1, round(duration * CONTROL_HZ))):
            yield


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.t = 0.0

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    @staticmethod
    def both(a, b):
        """Run two generators in lockstep until both end."""
        a, b = iter(a), iter(b)
        live = [a, b]
        while live:
            for g in list(live):
                try:
                    next(g)
                except StopIteration:
                    live.remove(g)
            if live:
                yield

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames: list[np.ndarray], path: Path) -> None:
    h, w = frames[0].shape[:2]
    ff = os.environ.get("FFMPEG", "ffmpeg")
    proc = subprocess.Popen(
        [ff, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(CONTROL_HZ),
         "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)],
        stdin=subprocess.PIPE,
    )
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(f[..., :3], dtype=np.uint8).tobytes())
    proc.stdin.close()
    proc.wait()


def run(task_path: str, out_dir: str, max_steps: int = 2400, cam_width: int = 320, cam_height: int = 176,
        hold_s: float = 2.0, record: bool = True, device: str = "cuda:0") -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height, device=device)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    ctx = Ctx(emb)
    script = emb.task.module.demo_script(ctx)
    frames: dict[str, list] = {}
    info: dict = {}
    steps, ended, success = 0, None, False

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    err = None
    try:
        while steps < max_steps:
            try:
                next(script)
            except StopIteration:
                ended = ended or steps
            res = emb.step(ctx.action())
            steps += 1
            ctx.t += 1.0 / CONTROL_HZ
            info = res.info
            snap(res.observation)
            if steps % 100 == 0:
                import time
                now = time.time()
                rate = 100 / max(now - getattr(run, "_t", now - 1e9), 1e-6)
                run._t = now
                print(f"step {steps} score {info.get('score')} {rate:.1f} steps/s", flush=True)
            if res.terminated:
                success = res.termination_reason == "success"
                break
            if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
                break
    except Exception as e:  # noqa: BLE001  (keep the videos of a failed run)
        import traceback

        traceback.print_exc()
        err = repr(e)
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"error": err, "task_score": score, "success": success, "steps": steps, "final_info": {k: v for k, v in info.items()
                                                                                       if isinstance(v, int | float | bool)}}
    if record:
        for cam, fr in frames.items():
            _write_mp4(fr, out / f"{cam}.mp4")
        import imageio

        imageio.imwrite(out / "setup.png", frames["top_cam"][0])
    (out / "result.json").write_text(json.dumps(result, indent=1))
    print("RESULT", json.dumps(result))
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    small = a.no_record  # debug runs: tiny camera frames, much faster rendering
    run(a.task, a.out, a.max_steps, record=not a.no_record, device=a.device,
        cam_width=64 if small else 320, cam_height=36 if small else 176)
