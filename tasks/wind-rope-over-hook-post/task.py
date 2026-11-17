"""Wind a rope over the post of a J-shaped rope hook: one hand holds one end in the air to the left of the hook, while
the other carries the rope round the front of the post and up over its top, three times, taking fresh rope further
along after each pass; both hands then hold the rope clear of the table. Scored on the passes over the post top made
while holding the rope, both hands holding at the end, and the rope off the table."""
from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
from cablekit import cable  # noqa: E402  (importing cablekit also hooks yam_arena so the rope is simulated)
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = (
    "Wind the rope over the hook's post: hold one end of the rope in the air with one hand, to the left of the hook. "
    "With the other hand, carry the rope round the front of the post and up over the post's top to its right side, "
    "three times, taking hold of the rope further along after each pass. End with both hands holding the rope and no "
    "part of the rope touching the table."
)

# -- J-shaped rope hook (world frame as in hang-cable-between-hooks: x forward from the arm bases, y left, z up) ----
# A grey wall panel stands across the table on the centre line. A square steel arm sticks out of it toward the robot,
# 10 cm above the table, and turns up through a rounded bend into a free-standing vertical post (16 mm diameter,
# 17 cm above the table) whose top is free. Nothing else stands within 6.5 cm of the post.
GREY = (0.6, 0.6, 0.6)
POST_X, POST_Y = 0.20, 0.0
POST_R = 0.008
ARM_Z = TOP + 0.10  # arm centre line
ARM_S = 0.012  # square arm, 12 x 12 mm
# The post runs from the bottom of the arm to 17 cm above the table; the right hand reaches only 18-20 cm above the
# table near the post, so a taller post could not be passed over.
POST_Z0, POST_Z1 = ARM_Z - ARM_S / 2, TOP + 0.17
PANEL_X0, PANEL_T = 0.33, 0.02  # panel front face and thickness

PANEL = Box("wall_panel", pos=(PANEL_X0 + PANEL_T / 2, POST_Y, TOP + 0.10), size=(PANEL_T, 0.24, 0.20),
            color=(0.75, 0.75, 0.72))
HOOK_ARM = Box("hook_arm", pos=((POST_X + PANEL_X0) / 2, POST_Y, ARM_Z), size=(PANEL_X0 - POST_X, ARM_S, ARM_S),
               color=GREY)
HOOK_BEND = Sphere("hook_bend", pos=(POST_X, POST_Y, ARM_Z), radius=ARM_S * 0.75, color=GREY)
HOOK_POST = Cylinder("hook_post", pos=(POST_X, POST_Y, (POST_Z0 + POST_Z1) / 2), radius=POST_R,
                     height=POST_Z1 - POST_Z0, color=GREY)

# -- rope: 100 cm, 12 mm, lying straight along y on the table in front of the hook, from y +37 cm to y -63 cm -------
R = 0.006
ROPE_LEN = 1.00
ROPE_X = 0.10
ROPE_Y0 = 0.37  # the left end
CORD = cable("rope", start=(ROPE_X, ROPE_Y0, TOP + R + 0.001), yaw=-math.pi / 2, length=ROPE_LEN, radius=R,
             color=(0.55, 0.35, 0.15), cable_mass=0.16, bend_stiffness=0.002, bend_damping=0.0005, max_bend=1.6,
             friction=0.5)
# Hemp rope: 160 g; soft bending, lightly damped; friction 0.5; up to 1.6 rad per 2 cm joint

OBJECTS = [PANEL, HOOK_ARM, HOOK_BEND, HOOK_POST, CORD]

# -- scoring ---------------------------------------------------------------------------------------------------------
TABLE_Z = TOP
PASSES = 3  # passes over the post top needed for full credit
HOLD_DIST = 0.03  # a gripper's TCP (between the finger pads) within this of the rope counts as holding it
OVER_IN = 0.03  # a holding gripper passes over the top: its TCP within 3 cm of the post's axis, in plan view, ...
OVER_Z = POST_Z1 + 0.005  # ... and at least 5 mm above the post's top
OVER_OUT = 0.06  # it must then move at least 6 cm from the axis before another pass can count
CLEAR_Z = TABLE_Z + R + 0.003  # off the table: every part of the rope at least 3 mm above it
SETTLE_STEPS = 20  # the full condition must hold for 1 continuous second at 20 Hz
_state = {"passes": 0, "over": [False, False], "full_steps": 0}


def _rope_pts(objects):
    return objects["rope"].polyline()


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def turns(objects, radius: float = 0.04) -> float:
    """Diagnostic only (not scored): net angle the rope sweeps about the post's axis, within 4 cm of it and along the
    post's height, divided by 2 pi."""
    pts = _dense(_rope_pts(objects))
    d = pts[:, :2] - (POST_X, POST_Y)
    near = (np.linalg.norm(d, axis=1) <= radius) & (pts[:, 2] >= POST_Z0 - 0.01) & (pts[:, 2] <= POST_Z1 + 0.01)
    ang = np.arctan2(d[:, 1], d[:, 0])
    step = (np.diff(ang) + np.pi) % (2 * np.pi) - np.pi
    return float(abs(step[near[:-1] & near[1:]].sum()) / (2 * np.pi))


def off_table(objects) -> bool:
    return bool(_rope_pts(objects)[:, 2].min() > CLEAR_Z)


# -- gripper positions, so the score can check that both hands hold the rope (as in hang-cable-between-hooks) -------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_coil_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._coil_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if the sim env is not known yet."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def _holding(objects) -> list[bool]:
    gs = grippers()
    if not gs:
        return [False, False]
    pts = _dense(_rope_pts(objects))
    return [float(np.linalg.norm(pts - g, axis=1).min()) < HOLD_DIST for g in gs]


def held(objects) -> bool:
    """Both grippers are within 3 cm of the rope (False while the gripper positions are unknown)."""
    return all(_holding(objects))


def _count_passes(objects) -> None:
    """A pass over the top: a gripper holding the rope comes within 3 cm of the post's axis (plan view) at least 5 mm
    above the post's top. It must move 6 cm or more from the axis before another pass by that gripper counts."""
    gs = grippers()
    if not gs:
        return
    hold = _holding(objects)
    for i, g in enumerate(gs):
        plan = float(np.hypot(g[0] - POST_X, g[1] - POST_Y))
        if not _state["over"][i] and hold[i] and plan < OVER_IN and g[2] > OVER_Z:
            _state["over"][i] = True
            _state["passes"] += 1
        elif _state["over"][i] and plan > OVER_OUT:
            _state["over"][i] = False


def score(objects):
    """Called once per control step (it counts the passes). With both hands holding the rope: 1.0 once it has been
    carried over the post top 3 or more times and has stayed clear of the table, still held, for 1 continuous second;
    0.75 for 3 or more passes otherwise; 0.5 for 2 passes; 0.25 for 1 pass; 0 for none. 0 if either hand is not
    holding the rope."""
    _count_passes(objects)
    n = _state["passes"]
    if not held(objects):
        _state["full_steps"] = 0
        return 0.0
    if n >= PASSES and off_table(objects):
        _state["full_steps"] += 1
        return 1.0 if _state["full_steps"] >= SETTLE_STEPS else 0.75
    _state["full_steps"] = 0
    return {0: 0.0, 1: 0.25, 2: 0.5}.get(n, 0.75)


def success(objects):
    return _state["full_steps"] >= SETTLE_STEPS and held(objects) and off_table(objects)


def distance(objects):
    """Passes over the post top still missing (0 once there are 3)."""
    return float(max(0, PASSES - _state["passes"]))


# -- closed-loop scripted demo (run with cabledemo.py) -------------------------------------------------------------
# The left hand takes the rope 3 cm from its left end and holds it 22 cm to the left of the post, 10 cm above the
# table (10 cm clear of the right hand's path). The right hand takes the rope 35 cm from the left end and, three
# times: carries it round the front of the post about 16 cm out, in close on the post's left side, up and over the
# top (20 cm above the table) to the right side, and out until the rope from the post top is taut. After passes 1
# and 2 it lets go and takes the rope again further along (58 cm, then 82 cm from the left end, at least 12 cm from the
# post); after pass 3 it keeps hold and both hands hold still. Waypoints were checked for reach (IK) and gripper
# clearance from the hook and the other gripper.

S_LEFT = 0.03  # arc length (m) from the left end where the left hand grips
GRIPS = (0.35, 0.58, 0.82)  # ... and where the right hand grips for passes 1, 2, 3
CARRY = TOP + 0.16
ANCHOR = (POST_X, POST_Y + 0.22, TOP + 0.10)  # left hand holds its end here throughout
FRONT = [(0.12, -0.14, TOP + 0.14), (0.06, -0.08, TOP + 0.14), (0.04, 0.0, TOP + 0.14), (0.06, 0.08, TOP + 0.14),
         (0.125, 0.065, TOP + 0.13)]  # round the front, ~16 cm from the post, to its left side
OVER = [(0.15, 0.065, TOP + 0.175), (0.17, 0.05, TOP + 0.185), (0.20, 0.0, TOP + 0.20), (0.20, -0.05, TOP + 0.20)]
R_OUT = 0.16  # how far out to the right the hand may go to pull the rope taut
YAW_HOME = {"left": -math.pi / 2, "right": math.pi / 2}  # each wrist's natural grip across a rope running along y
END_HOLD_S = 5.0  # still hold at the end


def _arc(pts):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]


def _at(pts, s):
    """Point and plan-view tangent angle of the rope at arc length ``s`` from its first point."""
    arc = _arc(pts)
    i = int(np.clip(np.searchsorted(arc, s) - 1, 0, len(pts) - 2))
    f = (s - arc[i]) / max(arc[i + 1] - arc[i], 1e-9)
    t = pts[i + 1] - pts[i]
    return pts[i] + f * t, math.atan2(t[1], t[0])


def _grip_yaw(theta, side):
    """Fingers close across the rope: yaw = rope direction mod pi, taken within pi/2 of the arm's natural yaw."""
    home = YAW_HOME[side]
    return (theta - home + math.pi / 2) % math.pi - math.pi / 2 + home


def _rope_from_left(ctx):
    pts = _rope_pts(ctx.states())
    return pts[::-1] if pts[0][1] < pts[-1][1] else pts  # arc lengths run from the left (+y) end


def demo_script(ctx):
    from yam_arena import ik, station
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import grip_rotation
    from yam_arena.ik import matrix_to_quat_xyzw

    L, Rt = ctx.left, ctx.right
    yl, yr = YAW_HOME["left"], YAW_HOME["right"]

    def log(tag):  # reads the state only: score() counts passes, so it is not called here
        st = ctx.states()
        p = _rope_pts(st)
        print(f"[demo t={ctx.t:5.1f}] {tag}: passes={_state['passes']} held={_holding(st)} off_table={off_table(st)} "
              f"rope_min_z={(p[:, 2].min() - TOP) * 100:.1f} cm turns={turns(st):.2f}", flush=True)

    def pick(arm, s, side, min_post):
        """The first rope point at or beyond arc length s (toward the free end) where the gripper fits and reaches: at
        least min_post from the post's axis, for the right hand at least 13 cm from the left hand, reachable by IK.
        Falls back to the point at s."""
        p = _rope_from_left(ctx)
        base = station.BASES[side]
        for s_try in np.arange(s, _arc(p)[-1] - 0.004, 0.005):
            g, th = _at(p, float(s_try))
            gz = max(float(g[2]), TOP + 0.008)
            if np.hypot(g[0] - POST_X, g[1] - POST_Y) < min_post:
                continue
            if arm is Rt and np.hypot(g[0] - ANCHOR[0], g[1] - ANCHOR[1]) < 0.13:
                continue
            yaw = _grip_yaw(th, side)
            _, pe, re = ik.solve((g[0], g[1], gz + 0.002), matrix_to_quat_xyzw(grip_rotation(yaw)), base_pos=base,
                                 iters=200)
            if pe < 0.004 and re < 0.05:
                return float(s_try), g, gz, yaw
        g, th = _at(p, s)
        return s, g, max(float(g[2]), TOP + 0.008), _grip_yaw(th, side)

    def grasp(arm, s, side, min_post=0.12, tries=3):
        """Hover above a clear, reachable rope point at or beyond s, let the arm settle, come down, close, lift 3 cm and
        check the rope came along; if not, open and try 3 cm further on. Returns the arc length gripped."""
        i = 0 if arm is L else 1
        for _ in range(tries):
            s_got, g, gz, yaw = pick(arm, s, side, min_post)
            print(f"    grasp {side}: wanted s={s:.3f}, gripping s={s_got:.3f}", flush=True)
            yield from arm.move((float(g[0]), float(g[1]), gz + 0.05), yaw=yaw, finger=FINGER_OPEN, duration=1.2)
            yield from arm.wait(0.5)
            yield from arm.move((float(g[0]), float(g[1]), gz + 0.002), yaw=yaw, duration=1.0)
            yield from arm.wait(0.3)
            yield from arm.grip(FINGER_CLOSED)
            yield from arm.move((float(g[0]), float(g[1]), gz + 0.03), yaw=yaw, duration=0.6)
            if _holding(ctx.states())[i]:
                return s_got
            print(f"    grasp {side}: missed at s={s_got:.3f}, retrying", flush=True)
            yield from arm.grip(FINGER_OPEN, duration=0.4)
            yield from arm.move((float(g[0]), float(g[1]), gz + 0.06), yaw=yaw, duration=0.5)
            s = s_got + 0.03
        return s_got

    def slack_over_top():
        """Rope length minus straight distance, from where the rope crosses the post top to the right gripper (None if
        the rope is not over the top)."""
        p = _dense(_rope_pts(ctx.states()))
        gs = grippers()
        if not gs:
            return None
        ig = int(np.argmin(np.linalg.norm(p - gs[1], axis=1)))
        top = np.where((np.hypot(p[:, 0] - POST_X, p[:, 1] - POST_Y) < 0.03) & (p[:, 2] > POST_Z1 - 0.01))[0]
        if len(top) == 0:
            return None
        it = int(top[np.argmin(np.abs(top - ig))])
        arc = _arc(p)
        return abs(arc[ig] - arc[it]) - float(np.linalg.norm(p[ig] - p[it]))

    def out_until_taut():
        """From above the post's right edge, go out along -y (and down to 15 cm) 1 cm at a time, up to R_OUT from the
        post; stop once the rope from the post top to the hand is within 1 cm of straight."""
        y = -0.05
        yield from Rt.move((POST_X, y, TOP + 0.17), yaw=yr, duration=0.6)
        while y > -R_OUT:
            sl = slack_over_top()
            if sl is not None and sl < 0.01:
                break
            y -= 0.01
            yield from Rt.move((POST_X, y, TOP + 0.17 - 0.02 * min(1.0, (-y - 0.05) / 0.05)), yaw=yr, duration=0.25)

    log("start")
    # 1) the left hand takes its end and holds it 22 cm to the left of the post, 10 cm up
    pts = _rope_from_left(ctx)
    pl, tl = _at(pts, S_LEFT)
    yield from L.move((pl[0], pl[1], CARRY), yaw=_grip_yaw(tl, "left"), finger=FINGER_OPEN, duration=1.5)
    yield from grasp(L, S_LEFT, "left", min_post=0.07)
    yield from L.move((pl[0], pl[1], CARRY), yaw=yl, duration=1.0)
    yield from L.move((ANCHOR[0], ANCHOR[1], CARRY), yaw=yl, duration=1.5)
    yield from L.move(ANCHOR, yaw=yl, duration=1.0)
    log("left hand in place")
    # 2) three passes over the post top; the right hand takes fresh rope before each one
    s = 0.0
    for n, s_want in enumerate(GRIPS):
        yield from Rt.move((0.12, -0.14, CARRY), yaw=yr, finger=FINGER_OPEN, duration=1.0)
        s_want = max(s_want, s + 0.05)  # never behind the previous grip (that rope is already on the post)
        s = yield from grasp(Rt, s_want, "right", min_post=0.07 if n == 0 else 0.12)  # at first the rope lies 10 cm out
        g = grippers()[1]
        yield from Rt.move((float(g[0]), float(g[1]), CARRY), yaw=yr, duration=1.0)
        yield from Rt.move((0.125, -0.07, CARRY), yaw=yr, duration=1.0)
        for wp in FRONT:
            yield from Rt.move(wp, yaw=yr, duration=0.8)
        for wp in OVER:
            yield from Rt.move(wp, yaw=yr, duration=1.0)
        yield from out_until_taut()
        yield from Rt.wait(0.5)
        log(f"pass {n + 1} over the top (right hand gripping {s * 100:.0f} cm from the left end)")
        if n == len(GRIPS) - 1:
            break
        yield from Rt.grip(FINGER_OPEN, duration=0.5)
        g = grippers()[1]
        yield from Rt.move((float(g[0]), float(g[1]), TOP + 0.18), yaw=yr, duration=0.6)
        yield from Rt.wait(1.0)
        log(f"pass {n + 1}: right hand let go")
    # 3) lift the rope clear of the table: the left hand's stretch sags onto the table, so the left hand rises, 1 cm
    #    at a time, up to 18 cm above the table, then moves further left, up to 28 cm from the post, until no part of the
    #    rope touches the table
    lz, ly = ANCHOR[2], ANCHOR[1]
    while not off_table(ctx.states()) and lz < TOP + 0.18 - 1e-6:
        lz += 0.01
        yield from L.move((ANCHOR[0], ly, lz), yaw=yl, duration=0.4)
    while not off_table(ctx.states()) and ly < POST_Y + 0.28 - 1e-6:
        ly += 0.01
        yield from L.move((ANCHOR[0], ly, lz), yaw=yl, duration=0.4)
    log(f"left hand at {(ly - POST_Y) * 100:.0f} cm left of the post, {(lz - TOP) * 100:.0f} cm up")
    # 4) both hands hold still
    yield from L.wait(END_HOLD_S)
    log("end")
