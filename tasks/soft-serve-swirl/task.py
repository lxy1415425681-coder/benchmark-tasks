"""Serve a soft-serve swirl into a plastic sundae cup at a countertop soft-serve machine.

One hand holds a 6 cm deep plastic sundae cup under the machine's nozzle; the other pulls the draw lever down to
dispense. While the serve flows, the cup is moved in circles under the nozzle and lowered, so the stream is laid down
in rings stacked into a swirl instead of a heap. The valve is self-closing: the serve flows only while the lever is
down, and the robot lets it up when the cup is full. The sim's machine holds one 40 cm (84 g) serving; letting the
lever up once it is all out breaks the stream off, and holding the lever down more than 1.5 s after that counts as
overfilling the cup (a real machine would keep flowing). The finished cup is set down upright on the round mat.

Scored from the serve's centre line, the cup's pose and the lever: the serving complete, every bit of serve in the
cup, how many rings the serve makes about the cup's axis, the cup filled to the rim, and the cup set down upright on
the mat and let go. Serve on the table or the machine, the cup set down on the table while the serve is flowing, or
the cup knocked over ends in 0.

Parts come from mechkit.py (machine, sprung lever, feed piston, cup) and cablekit.py (the serve: a chain of short
capsules); see README.md.
"""
from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402  (machine, lever, piston, cup; installed in memory)
from cablekit import cable  # noqa: E402  (the serve; installed in memory)
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0

INSTRUCTION = (
    "Serve a soft-serve swirl in the sundae cup: hold the cup under the machine's nozzle with one hand and pull the "
    "lever down with the other. While it flows, move the cup in circles under the nozzle and lower it slowly, so the "
    "serve is laid in rings stacked up to the cup's rim. When the cup is full, let the lever up, then set the cup down "
    "upright on the green mat."
)

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
X_N, Y_N = 0.33, -0.09  # the nozzle's axis
NOZ_TIP = TOP + 0.25  # bottom of the nozzle
HEAD_BOT = TOP + 0.28  # underside of the machine's head
CH = 0.0125  # half width of the feed channel above the nozzle (2.5 cm square: the 19 mm serve cannot buckle in it)
NOZ_IN = 0.0125  # half width of the nozzle bore

STEEL = (0.78, 0.79, 0.80)
DARK = (0.18, 0.18, 0.20)
CHROME = (0.62, 0.63, 0.66)

kit = mk.Kit()

# -- machine (fixed): lower body behind the cup space, upper body with the head over the cup space, nozzle ----------
BODY_X1 = X_N + 0.32
BODY_Y0, BODY_Y1 = Y_N - 0.12, Y_N + 0.19  # the machine's sides
LOW_X0 = X_N + 0.075  # front of the lower body: 7.5 cm behind the nozzle axis
TOP_Z = TOP + 0.80
FRONT_X0 = X_N - 0.028  # front face of the head


def _slab(x0, x1, y0, y1, z0, z1, color=None):
    return mk.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color)


_machine = [
    _slab(LOW_X0, BODY_X1, BODY_Y0, BODY_Y1, TOP, HEAD_BOT),  # lower body
    # upper body around the feed channel (a 2.5 cm square shaft straight above the nozzle)
    _slab(FRONT_X0, X_N - CH, BODY_Y0, BODY_Y1, HEAD_BOT, TOP_Z),  # front panel
    _slab(X_N + CH, BODY_X1, BODY_Y0, BODY_Y1, HEAD_BOT, TOP_Z),  # back block
    _slab(X_N - CH, X_N + CH, Y_N + CH, BODY_Y1, HEAD_BOT, TOP_Z),  # left of the channel
    _slab(X_N - CH, X_N + CH, BODY_Y0, Y_N - CH, HEAD_BOT, TOP_Z),  # right of the channel
    _slab(X_N - CH, X_N + CH, Y_N - CH, Y_N + CH, TOP_Z - 0.01, TOP_Z),  # channel cap
    # nozzle: a square spout 3 cm long below the head, 2.5 cm bore
    _slab(X_N - NOZ_IN - 0.004, X_N - NOZ_IN, Y_N - NOZ_IN - 0.004, Y_N + NOZ_IN + 0.004, NOZ_TIP, HEAD_BOT, CHROME),
    _slab(X_N + NOZ_IN, X_N + NOZ_IN + 0.004, Y_N - NOZ_IN - 0.004, Y_N + NOZ_IN + 0.004, NOZ_TIP, HEAD_BOT, CHROME),
    _slab(X_N - NOZ_IN, X_N + NOZ_IN, Y_N + NOZ_IN, Y_N + NOZ_IN + 0.004, NOZ_TIP, HEAD_BOT, CHROME),
    _slab(X_N - NOZ_IN, X_N + NOZ_IN, Y_N - NOZ_IN - 0.004, Y_N - NOZ_IN, NOZ_TIP, HEAD_BOT, CHROME),
]
# lever bracket on the head's front face
Y_LEV = Y_N + 0.15  # 15 cm left of the nozzle: room for both hands
PIVOT = (FRONT_X0 - 0.008, Y_LEV, TOP + 0.32)
_machine.append(_slab(FRONT_X0 - 0.016, FRONT_X0, Y_LEV - 0.018, Y_LEV + 0.018, TOP + 0.305, TOP + 0.335, DARK))
# Polished steel, and the soft serve slides on it: friction 0.02, combined as the lower of the pair, so the serve never
# wedges in the feed channel or the nozzle.
kit.part("machine", (0.0, 0.0, 0.0), _machine, color=STEEL, friction=(0.02, 0.02))

# -- draw lever: 10 cm arm forward from the pivot with a knob; pulled down it opens the valve ---------------------
LEV_LEN = 0.10
LEV_DOWN_DEG = 32.0  # down stop
VALVE_DEG = 12.0  # the valve opens past this ...
FULL_DEG = 25.0  # ... and is fully open past this
kit.part("lever", PIVOT, [
    mk.box((LEV_LEN, 0.014, 0.014), at=(-LEV_LEN / 2, 0.0, 0.0), color=DARK),
    mk.box((0.030, 0.026, 0.026), at=(-LEV_LEN - 0.010, 0.0, 0.0), color=(0.75, 0.12, 0.12)),
], mass=0.08, color=DARK)
# Hinge about +y: pulling the knob down is a negative angle. Spring back to (and preloaded against) the up stop.
kit.hinge("lever_hinge", "machine", "lever", anchor=PIVOT, axis=(0, 1, 0), lower_deg=-LEV_DOWN_DEG, upper_deg=0.0,
          start_deg=0.0, stiffness=0.5, rest_deg=8.0, damping=0.02)

# -- the serve: 43 cm of 19 mm "rope" hanging in the channel, its tip 5 mm inside the nozzle -----------------------
SERVE_R = 0.0095
SERVE_LEN = 0.43  # 40 cm is served (FEED_END); the last 3 cm stays in the machine's feed
SERVE_KG = 0.090  # the whole 43 cm strand: about 0.74 g/cm^3 (soft serve at about 45% overrun); the 40 cm serving is 84 g
SERVE_TOP = NOZ_TIP + 0.005 + SERVE_LEN  # its upper end at the start
PISTON = 0.020
PISTON_KG = 0.05
PISTON_C = (X_N, Y_N, SERVE_TOP + PISTON / 2)
kit.part("piston", PISTON_C, [mk.box((PISTON, PISTON, PISTON))], mass=PISTON_KG, color=DARK)
FEED_END = SERVE_LEN - 0.03  # the stream is cut (upper end let go) once this much is out: the rest drops out
kit.slider("feed", "machine", "piston", axis=(0, 0, -1), lower=0.0, upper=SERVE_LEN - 0.02, start=0.0,
           stiffness=4000.0, rest=0.0, damping=80.0)
FEED_SPARE_N = 0.3  # the feed drive's force limit is the weight it holds plus this: it neither rams the serve nor holds
# the stream back when it is pulled (the stream from the nozzle thins and stretches rather than pulling the cup)
FEED_RATE = 0.05  # m/s with the lever fully open: the 40 cm serving takes 8 s
OVERRUN_S = 1.5  # the lever held open this long after the serving is all out = overfilled: a real machine keeps
# flowing, and 1.5 s at full flow adds about 7.5 cm (16 g), past the 100 g overfill limit

SERVE = cable("serve", start=(X_N, Y_N, SERVE_TOP), pitch=math.pi / 2, length=SERVE_LEN, radius=SERVE_R,
              segment=0.012, color=(0.97, 0.94, 0.84), cable_mass=SERVE_KG, bend_stiffness=0.00003,
              bend_damping=0.002, max_bend=1.2, friction=1.2, collision_sides=-1, anchor_body="piston",
              anchor_local=(0.0, 0.0, -PISTON / 2))

# -- plastic sundae cup: 6 cm deep, 6.4 cm across inside at the bottom and 7.0 cm at the rim (210 ml); 12 flat facets.
# The 40 cm serving laid in three 2.1 cm rings, each a serve's thickness (1.9 cm) high, fills it to about the rim.
CUP_START = (0.26, -0.18)
CUP_H = 0.060
CUP_RT, CUP_RB = 0.035, 0.032  # inner radius at the rim and at the bottom
WALL = 0.0025
N_FACETS = 12
CUP_KG = 0.050  # a reusable rigid plastic sundae cup (a 12 g paper cup under 80 g of serve welded into it jumps when
# the hand lets go: the welds and the wall's contacts fight, and only a firm grip holds a light cup still)


def cup_inner_r(z):
    """The cup's inner radius at height z above its base (clipped to the cup)."""
    return CUP_RB + (CUP_RT - CUP_RB) * np.clip(np.asarray(z, dtype=float) / CUP_H, 0.0, 1.0)
CUP_TILT = math.atan2(CUP_RT - CUP_RB, CUP_H)


def _cup_boxes():
    out = []
    slant = CUP_H / math.cos(CUP_TILT)
    w = 2 * (CUP_RT + WALL) * math.tan(math.pi / N_FACETS) + 0.001
    for k in range(N_FACETS):
        th = 2 * math.pi * k / N_FACETS
        r = (CUP_RT + CUP_RB) / 2 + WALL / 2
        rot = mk.quat_to_mat(mk.yaw_quat(th)) @ mk.axis_angle((0, 1, 0), CUP_TILT)
        out.append(mk.box((WALL, w, slant), at=(r * math.cos(th), r * math.sin(th), CUP_H / 2),
                          quat=tuple(mk.mat_to_quat(rot))))
    for yaw in (0.0, math.pi / 4):
        s = 2 * CUP_RB * math.cos(math.pi / 8)
        out.append(mk.box((s, s, WALL), at=(0.0, 0.0, WALL / 2), yaw=yaw))
    return out


kit.part("cup", (CUP_START[0], CUP_START[1], TOP), _cup_boxes(), mass=CUP_KG, color=(0.93, 0.93, 0.95))

# -- serving mat: a green rubber disc, 12 cm across ----------------------------------------------------------------
MAT_XY = (0.17, -0.09)
MAT_R = 0.06
MAT = Cylinder("mat", pos=(MAT_XY[0], MAT_XY[1], TOP + 0.001), radius=MAT_R, height=0.002, color=(0.25, 0.6, 0.3))

OBJECTS = [*kit.objects(), SERVE, MAT]


# -- the machine: feed control, stream cut (sim only; per-episode state in mk.MEMORY) ------------------------------
def lever_deg(objects) -> float:
    """How far the lever is pulled down (deg, >= 0)."""
    return max(0.0, -kit.q_deg(objects, "lever_hinge"))


def valve(objects) -> float:
    """Valve opening 0..1 from the lever angle."""
    return float(np.clip((lever_deg(objects) - VALVE_DEG) / (FULL_DEG - VALVE_DEG), 0.0, 1.0))


def _feed_joint_path():
    return mk._SIM["joint_paths"].get((kit.id, "feed"))


def _anchor_path(gen: int) -> str:
    env = mk._SIM["paths"]["machine"].rsplit("/", 1)[0]
    return f"{env}/serve/anchor_body" + (f"_r{gen}" if gen else "")


def _machine_step(objects) -> None:
    from pxr import UsdPhysics

    stage = mk._SIM["stage"]
    m = mk.MEMORY
    if "feed" not in m:  # first step of an episode: weld the serve back on (it was cut last episode) and zero the feed
        g = mk._SIM.setdefault("serve_gen", 0)
        if mk._SIM.get("serve_cut"):
            import cablekit

            stage.RemovePrim(_anchor_path(g))
            mk._SIM["serve_gen"] = g = g + 1
            prim_path = mk._SIM["paths"]["machine"].rsplit("/", 1)[0] + "/serve"
            cablekit._author_body_anchor(stage, prim_path, SERVE, path=_anchor_path(g))
            mk._SIM["serve_cut"] = False
        m.update(feed=0.0, cut=False, flow_steps=0, flow_on_table=False, cup_low=False, knocked=False, on_table=False,
                 overrun_steps=0, overrun=False)
    v = valve(objects)
    if not m["cut"] and v > 0:
        if m["feed"] >= FEED_END - 1e-9:  # the serving is all out and the lever is still open: a real machine overfills
            m["overrun_steps"] += 1
            if m["overrun_steps"] > OVERRUN_S * CONTROL_HZ:
                m["overrun"] = True
        m["feed"] = min(FEED_END, m["feed"] + FEED_RATE * v / CONTROL_HZ)
        m["flow_steps"] += 1
        if cup_on_table(objects):
            m["flow_on_table"] = True
    path = _feed_joint_path()
    if path:
        drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(path), "linear")
        drive.GetTargetPositionAttr().Set(float(m["feed"]))
    m["travel"] = travel = float(PISTON_C[2] - objects["piston"].pos[2])
    if path:
        held = PISTON_KG + SERVE_KG * max(0.0, SERVE_LEN - travel) / SERVE_LEN
        drive.CreateMaxForceAttr().Set(float(held * 9.81 + FEED_SPARE_N))
    # self-closing valve: the stream breaks off at the nozzle when the lever is let up once the serving is all out
    # (letting it up earlier only pauses the flow)
    if not m["cut"] and m["feed"] >= FEED_END - 1e-9 and travel >= FEED_END - 0.005 and v <= 0:
        m["cut"] = True
        # let PhysX break the weld (a tiny break force) rather than switching the joint off under a live articulation
        UsdPhysics.Joint(stage.GetPrimAtPath(_anchor_path(mk._SIM["serve_gen"]))).CreateBreakForceAttr().Set(1e-4)
        mk._SIM["serve_cut"] = True


_DUMP: list = []


def _dump_step(objects) -> None:
    """Debug (SERVE_DUMP=path): keep the serve, cup and piston every step; written by dump_states()."""
    import os

    if not os.environ.get("SERVE_DUMP"):
        return
    pts = serve_pts(objects)
    _DUMP.append(dict(serve=pts.copy(), cup=_cup_T(objects).copy(), piston=np.asarray(objects["piston"].pos),
                      feed=mk.MEMORY.get("feed", 0.0), cut=bool(mk.MEMORY.get("cut")), lever=lever_deg(objects),
                      tcps=np.array(mk.tcps()) if mk.tcps() else np.zeros((2, 3))))
    if len(_DUMP) % 50 == 0 or not np.isfinite(pts).all():
        np.save(os.environ["SERVE_DUMP"], np.array(_DUMP, dtype=object), allow_pickle=True)


# -- stickiness: soft serve stays where it is laid -------------------------------------------------------------------
# A chain of beads has no cohesion, so serve laid in rings slides off them. Real soft serve is stiff and sticky: a ring
# stays where it lands. So a serve segment is welded where it lies to the cup (a fixed joint) when (a) it has just left
# the nozzle over the cup and is within LAY_GAP of the floor or of serve already stuck below it (laid like a bead from a
# piping bag), or (b) it touches the cup's floor or wall, or rests on serve already stuck, moving slower than
# STICK_SPEED. Serve that lands anywhere else stays loose.
STICK_GAP = 0.006  # contact: within this of the floor or wall (beyond the serve radius) or of a stuck segment (beyond
# 2 radii)
STICK_SPEED = 0.20  # m/s relative to the cup: slower sticks on touching (the stream flows at 0.05; soft serve is
# sticky and stays where it is laid, so only a piece flung or falling fast does not)
STICK_BREAK_N = 3.0
LAY_GAP = 0.012  # serve leaving the nozzle is laid where it is if it is at most this far above what it would rest on
# (the floor or serve already stuck): soft serve is stiff enough to bridge a gap this small, not more


def support_top(loc_stuck: np.ndarray, xy, idx_stuck=None, seg: int | None = None) -> float:
    """Height of the top of what serve at ``xy`` (cup frame) would rest on: stuck serve within 1.5 serve radii of it
    horizontally, else the cup's floor. For segment ``seg``, its own neighbours along the serve (within 4 segments,
    laid just before it) are not support."""
    top = WALL
    if len(loc_stuck):
        near = np.hypot(loc_stuck[:, 0] - xy[0], loc_stuck[:, 1] - xy[1]) < 1.5 * SERVE_R
        if seg is not None:
            near &= np.abs(np.asarray(idx_stuck) - seg) > 4
        if near.any():
            top = max(top, float(loc_stuck[near, 2].max()) + SERVE_R)
    return top


def _seg_quats() -> np.ndarray:
    base = mk._EP["env"].unwrapped
    art = base.scene["serve"]
    q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0]
    names = list(art.body_names)
    idx = [names.index(f"seg_{i}") for i in range(SERVE.segments)]
    return q[idx].detach().cpu().numpy().astype(float)  # (n, 4), Isaac Lab 3: xyzw


def _stick_step(objects) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    stage, m = mk._SIM["stage"], mk.MEMORY
    if "stuck" not in m:
        for path in mk._SIM.get("stick_paths", []):
            stage.RemovePrim(path)
        mk._SIM["stick_paths"] = []
        mk._SIM["stick_gen"] = mk._SIM.get("stick_gen", 0) + 1
        m["stuck"], m["prev_local"] = set(), None
    t = _cup_T(objects)
    pts = objects["serve"].points
    loc = (pts - t[:3, 3]) @ t[:3, :3]
    prev, m["prev_local"] = m["prev_local"], loc
    if prev is None or cup_tilt_deg(objects) > 30.0:
        return
    speed = np.linalg.norm(loc - prev, axis=1) * CONTROL_HZ
    r = np.hypot(loc[:, 0], loc[:, 1])
    # in the cup (clear of the wall), or above the rim within the rim line (it can only stick there on serve)
    r_max = np.where(loc[:, 2] < CUP_H, cup_inner_r(loc[:, 2]) - SERVE_R + 0.002, CUP_RT)
    in_cup = (r <= r_max) & (loc[:, 2] > 0.0) & (pts[:, 2] < NOZ_TIP - 0.01)
    stuck = m["stuck"]
    new = []
    # laid as it leaves the nozzle: just out of the nozzle, over the cup and close above what it would rest on
    st_idx = sorted(stuck)
    loc_st = loc[st_idx] if st_idx else np.zeros((0, 3))
    fresh = ((pts[:, 2] < NOZ_TIP - 0.002) & (pts[:, 2] > NOZ_TIP - 0.03)
             & (np.hypot(pts[:, 0] - X_N, pts[:, 1] - Y_N) < NOZ_IN + 0.006) & (r <= np.maximum(r_max, CUP_RT)))
    for i in np.where(fresh)[0]:
        i = int(i)
        if i not in stuck and loc[i, 2] - SERVE_R - support_top(loc_st, loc[i, :2], st_idx, i) <= LAY_GAP:
            new.append(i)
    for i in np.where(in_cup & (speed < STICK_SPEED))[0]:
        i = int(i)
        if i in stuck or i in new:
            continue
        on_floor = loc[i, 2] < WALL + SERVE_R + STICK_GAP or (r[i] > cup_inner_r(loc[i, 2]) - SERVE_R - 0.0005 and loc[i, 2] < CUP_H)
        # resting on serve laid earlier (below it, not beside it; not its own neighbours along the serve)
        on_serve = any(abs(i - j) > 4 and loc[j, 2] < loc[i, 2] - 0.5 * SERVE_R
                       and np.linalg.norm(loc[i] - loc[j]) < 2 * SERVE_R + STICK_GAP for j in stuck)
        if on_floor or on_serve:
            new.append(i)
    if not new:
        return
    quats = _seg_quats()
    env = mk._SIM["paths"]["cup"].rsplit("/", 1)[0]
    rc = t[:3, :3]
    for i in new:
        rs = mk.quat_to_mat(quats[i])
        path = f"{env}/serve/stick_{mk._SIM['stick_gen']}_{i}"
        j = UsdPhysics.FixedJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{env}/cup")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{env}/serve/seg_{i}")])
        # joint frame: the segment's pose now, expressed on the cup (the cup's origin is its prim's frame)
        x, y, z, w = mk.mat_to_quat(rc.T @ rs)
        j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in loc[i]]))
        j.CreateLocalRot0Attr(Gf.Quatf(float(w), float(x), float(y), float(z)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateCollisionEnabledAttr().Set(False)
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateBreakForceAttr().Set(STICK_BREAK_N)  # pulled harder, the serve tears off rather than yanking the cup
        mk._SIM["stick_paths"].append(path)
        stuck.add(i)


# -- look: the serve is drawn as one smooth tube through its centre line instead of a chain of capsules (display
# only: the capsules stay as they are for physics, just hidden) -----------------------------------------------------
TUBE_SIDES = 16


def _smooth(c: np.ndarray, passes: int = 2) -> np.ndarray:
    """Chaikin corner cutting, keeping the ends."""
    for _ in range(passes):
        q = 0.75 * c[:-1] + 0.25 * c[1:]
        r = 0.25 * c[:-1] + 0.75 * c[1:]
        c = np.vstack([c[:1], np.stack([q, r], axis=1).reshape(-1, 3), c[-1:]])
    return c


def _tube_step(objects) -> None:
    from pxr import Gf, UsdGeom, Vt

    stage = mk._SIM["stage"]
    env = mk._SIM["paths"]["cup"].rsplit("/", 1)[0]
    path = "/World/serve_tube"
    st = objects["serve"]
    c = np.asarray(st.points, dtype=float)
    seg = st.seg_len
    c = np.vstack([c[0] + st.start_dir * seg / 2, c, c[-1] + st.end_dir * seg / 2])  # out to the ends
    c = _smooth(c) + mk._SIM["env_T"][:3, 3]
    n, k = len(c), TUBE_SIDES
    t = np.gradient(c, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    # parallel-transport frames (no twisting)
    a = np.cross(t[0], (0.0, 0.0, 1.0) if abs(t[0][2]) < 0.9 else (1.0, 0.0, 0.0))
    a /= np.linalg.norm(a)
    us = [a]
    for i in range(1, n):
        u = us[-1] - t[i] * np.dot(us[-1], t[i])
        nu = np.linalg.norm(u)
        us.append(u / nu if nu > 1e-9 else us[-1])
    u = np.array(us)
    v = np.cross(t, u)
    ang = 2 * np.pi * np.arange(k) / k
    ring = (c[:, None, :] + SERVE_R * (np.cos(ang)[None, :, None] * u[:, None, :]
                                       + np.sin(ang)[None, :, None] * v[:, None, :]))
    pts = np.vstack([ring.reshape(-1, 3), c[0], c[-1]])
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    if not mesh:
        mesh = UsdGeom.Mesh.Define(stage, path)
        counts, idx = [], []
        for i in range(n - 1):
            for j in range(k):
                j2 = (j + 1) % k
                counts.append(4)
                idx += [i * k + j, i * k + j2, (i + 1) * k + j2, (i + 1) * k + j]
        for j in range(k):  # end caps
            counts += [3, 3]
            idx += [n * k, (j + 1) % k, j, n * k + 1, (n - 1) * k + j, (n - 1) * k + (j + 1) % k]
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        from isaaclab.sim.utils import bind_visual_material

        bind_visual_material(path, f"{env}/serve/Looks/cable", stage=stage)
        for i in range(SERVE.segments):  # hide the capsules (display only)
            UsdGeom.Imageable(stage.GetPrimAtPath(f"{env}/serve/seg_{i}/geom")).MakeInvisible()
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*pts.min(0).tolist()), Gf.Vec3f(*pts.max(0).tolist())]))


kit.on_step(_machine_step)
kit.on_step(_tube_step)
kit.on_step(_stick_step)
kit.on_step(_dump_step)


# -- geometry for scoring ------------------------------------------------------------------------------------------
def _cup_T(objects) -> np.ndarray:
    pos, q = kit.pose(objects, "cup")
    return mk._T(mk.quat_to_mat(q), pos)


def cup_tilt_deg(objects) -> float:
    r = _cup_T(objects)[:3, :3]
    return float(math.degrees(math.acos(np.clip(r[2, 2], -1.0, 1.0))))


def cup_on_table(objects) -> bool:
    """The cup's base rests on the table: its origin (bottom centre) within 5 mm of the table top."""
    return bool(_cup_T(objects)[2, 3] < TOP + 0.005)


def serve_pts(objects) -> np.ndarray:
    return objects["serve"].polyline()


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _in_cup_frame(objects, pts) -> np.ndarray:
    t = _cup_T(objects)
    return (pts - t[:3, 3]) @ t[:3, :3]


OVERHANG = 0.015  # above the rim, serve may overhang the rim line by up to this; farther out it is falling off


def inside_cup(objects) -> np.ndarray:
    """Per dense centre-line point: in the cup, or on top of it. Below the rim plane a point must be inside the cup's
    outer wall (centre within the wall plus the serve radius plus 2 mm: serve resting on the outside of the wall is
    outside); above it, within OVERHANG of the rim line."""
    p = _in_cup_frame(objects, _dense(serve_pts(objects)))
    r = np.hypot(p[:, 0], p[:, 1])
    rim = cup_inner_r(p[:, 2])
    below = p[:, 2] < CUP_H
    ok_below = r <= rim + WALL + SERVE_R + 0.002
    ok_above = r <= rim + OVERHANG
    return np.where(below, ok_below, ok_above) & (p[:, 2] > -0.005)


def serve_out_of_machine(objects) -> bool:
    """The whole serve has left the nozzle (the stream was cut and nothing is above the nozzle tip)."""
    return bool(mk.MEMORY.get("cut")) and bool(serve_pts(objects)[:, 2].max() < NOZ_TIP)


def on_table_outside(objects) -> bool:
    """Serve outside the cup within 3 mm of resting on the table."""
    pts = _dense(serve_pts(objects))
    return bool(((pts[:, 2] < TOP + SERVE_R + 0.003) & ~inside_cup(objects)).any())


def touches_outside(objects) -> bool:
    """Serve on the table, the machine or outside the cup: once the stream is cut, any centre-line point outside the
    cup and its rim's footprint; at any time, a point outside the cup that is within 3 mm of resting on the table."""
    pts = _dense(serve_pts(objects))
    inside = inside_cup(objects)
    if bool(((pts[:, 2] < TOP + SERVE_R + 0.003) & ~inside).any()):
        return True
    return bool(mk.MEMORY.get("cut")) and not bool(inside.all())


def rings(objects) -> float:
    """Net turns the serve's centre line makes about the cup's axis (inside the cup and its rim's footprint), in a
    consistent direction: |sum of signed angle steps| / 2 pi."""
    p = _in_cup_frame(objects, _dense(serve_pts(objects)))
    ok = inside_cup(objects)
    ang = np.arctan2(p[:, 1], p[:, 0])
    r = np.hypot(p[:, 0], p[:, 1])
    step = (np.diff(ang) + np.pi) % (2 * np.pi) - np.pi
    use = ok[:-1] & ok[1:] & (r[:-1] > 0.004) & (r[1:] > 0.004)  # near the axis the angle is meaningless
    return float(abs(step[use].sum()) / (2 * math.pi))


def height_above_rim(objects) -> float:
    """Height of the top of the serve above the rim plane (m, along the cup's axis)."""
    p = _in_cup_frame(objects, serve_pts(objects))
    return float(p[:, 2].max() + SERVE_R - CUP_H)


def on_mat(objects) -> bool:
    t = _cup_T(objects)
    return bool(np.hypot(t[0, 3] - MAT_XY[0], t[1, 3] - MAT_XY[1]) <= MAT_R - CUP_RB and t[2, 3] < TOP + 0.008)


def released(objects) -> bool:
    return mk.released(objects["cup"], margin=0.02)


def lever_up(objects) -> bool:
    return lever_deg(objects) < VALVE_DEG


# -- score ---------------------------------------------------------------------------------------------------------
RINGS_FULL, RINGS_PART = 2.5, 1.5
HEIGHT_FULL = -0.01  # the cup filled: the top of the swirl no more than 1 cm below the rim (and nothing over the edge)
SETTLE_STEPS = 20


def _base_score(objects) -> float:
    m = mk.MEMORY
    # "at any time": a cup tipped past 30 deg or serve on the table is remembered for the rest of the episode
    if cup_tilt_deg(objects) > 30.0:
        m["knocked"] = True
    if on_table_outside(objects):
        m["on_table"] = True
    if (m.get("knocked") or m.get("on_table") or m.get("flow_on_table") or m.get("overrun")
            or touches_outside(objects)):
        return 0.0
    if not serve_out_of_machine(objects):
        return 0.0 if not m.get("feed") else 0.1
    n, h = rings(objects), height_above_rim(objects)
    shaped = n >= RINGS_FULL and h >= HEIGHT_FULL
    if shaped and on_mat(objects) and cup_tilt_deg(objects) <= 10.0 and released(objects) and lever_up(objects):
        return 1.0
    if shaped:
        return 0.75
    if n >= RINGS_PART:
        return 0.5
    return 0.25


def score(objects):
    """Called every control step. 0: the cup knocked over (> 30 deg), serve on the table, the machine or outside the
    cup, or serve flowed while the cup stood on the table. Serving not finished: 0.1 once any has flowed, else 0.
    Serving finished and all in the cup: 0.25; with 1.5 or more rings, 0.5; with 2.5 or more rings and the cup filled
    (the top of the serve no more than 1 cm below the rim), 0.75; and that, with the cup upright (within 10 deg) on the mat, let go and the lever up, for
    1 continuous second, 1.0."""
    s = _base_score(objects)
    m = mk.MEMORY
    if s >= 1.0:
        m["full_steps"] = m.get("full_steps", 0) + 1
        return 1.0 if m["full_steps"] >= SETTLE_STEPS else 0.75
    m["full_steps"] = 0
    return s


def success(objects):
    return mk.MEMORY.get("full_steps", 0) >= SETTLE_STEPS


def distance(objects):
    return float(np.hypot(*(_cup_T(objects)[:2, 3] - MAT_XY)))


# -- closed-loop scripted demo (run with servedemo.py) ---------------------------------------------------------------
# The right hand takes the cup from the side (fingers closing across it, approach tilted 25 deg down, 2 cm under the
# rim) and holds it with its rim 5 mm below the nozzle. The left hand takes the lever's knob and pulls it down to its
# stop along its arc. While the serve flows, the right hand moves the cup in 2.1 cm circles about the nozzle at a fixed
# rate (three turns over the 8 s serving) and lowers it steadily, 1.6 cm in all. When the serving is all out, the
# left hand lets the lever up and the right hand lowers the cup, draws it out from under the head, sets it down on the
# mat, eases its grip, opens and backs away.
CUP_GRIP_Z = 0.040  # TCP height above the cup's base when held: 2 cm under the rim, above the full cup's centre of
# mass (about mid-height), so the cup hangs upright in the grip instead of tipping about the pads
CUP_PITCH = 25.0
CUP_FINGER = -0.030  # finger target holding the cup: 8 mm past contact (-0.038 at 4 cm up, measured), so each finger presses with a
# spring force (~40 N) that grows if the cup drifts toward it; closing fully saturates both at 50 N and the cup drifts
LEV_PITCH = 30.0
KNOB_R = LEV_LEN + 0.010  # pivot to knob centre
HOLD_RIM_GAP = 0.005  # rim below the nozzle tip at the start
SWIRL_R = FEED_END / (3 * 2 * math.pi)  # 2.1 cm: three rings take the whole 40 cm serving
SWIRL_W = FEED_RATE / SWIRL_R  # rad/s: the circle keeps pace with the stream at full flow (3 turns in 8 s)
LOWER_RATE = 0.002  # m/s, steady: 1.6 cm over the serving, so the nozzle ends about a serve's thickness above the top
HOLD_AFTER_S = 1.0  # after the cut, hold still while the last of the serve drops out
PULL_DEG = 40.0  # past the 32 deg stop: the stop sets the angle, whatever the arm's sag


def grasp_cup(ctx, tries: int = 4):
    """Right hand: take the cup from the side, fingers closing across it, and check it sits centred between the pads
    (within 5 mm across the fingers); if not, open, line up on where the cup now is and close again."""
    from yam_arena.constants import FINGER_OPEN

    Rt = ctx.right
    t = _cup_T(ctx.states())
    cx, cy = float(t[0, 3]), float(t[1, 3])
    yield from Rt.move((cx - 0.10, cy, TOP + 0.10), pitch=CUP_PITCH, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((cx - 0.10, cy, TOP + CUP_GRIP_Z), pitch=CUP_PITCH, duration=1.0)
    for k in range(tries):
        yield from Rt.move((cx, cy, TOP + CUP_GRIP_Z), pitch=CUP_PITCH, duration=1.2)
        yield from Rt.wait(0.3)
        yield from Rt.grip(CUP_FINGER, duration=1.0)
        yield from Rt.wait(0.3)
        t = _cup_T(ctx.states())
        err = float(mk.tcps()[1][1] - t[1, 3])  # across the fingers (world y)
        print(f"    cup grasp {k + 1}: off centre {err * 1000:.0f} mm across the fingers", flush=True)
        if abs(err) <= 0.005:
            return
        yield from Rt.grip(FINGER_OPEN, duration=0.6)
        t = _cup_T(ctx.states())
        cx, cy = float(t[0, 3]), float(t[1, 3])
        yield from Rt.move((cx - 0.06, cy, TOP + CUP_GRIP_Z), pitch=CUP_PITCH, duration=1.0)


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    m = mk.MEMORY
    cx, cy = CUP_START
    hold_z = NOZ_TIP - HOLD_RIM_GAP - CUP_H + CUP_GRIP_Z

    def knob(th_deg):
        t = math.radians(th_deg)
        return (PIVOT[0] - KNOB_R * math.cos(t), PIVOT[1], PIVOT[2] - KNOB_R * math.sin(t))

    def log(tag):
        st = ctx.states()
        print(f"[demo t={ctx.t:5.1f}] {tag}: lever={lever_deg(st):.1f} feed={m.get('feed', 0) * 100:.1f}cm travel={m.get('travel', 0) * 100:.1f}cm "
              f"cut={m.get('cut')} rings={rings(st):.2f} h_above_rim={height_above_rim(st) * 100:.1f}cm "
              f"tilt={cup_tilt_deg(st):.1f} outside={touches_outside(st)}", flush=True)

    def right_take_cup():
        yield from grasp_cup(ctx)
        g = mk.tcps()[1]
        yield from Rt.move((float(g[0]), float(g[1]), hold_z), pitch=CUP_PITCH, duration=1.2)
        yield from Rt.move((X_N - 0.07, Y_N, hold_z - 0.01), pitch=CUP_PITCH, duration=1.2)
        yield from Rt.move((X_N, Y_N, hold_z), pitch=CUP_PITCH, duration=1.0)

    def left_take_lever():
        k = knob(0.0)
        yield from L.move((k[0] - 0.05, k[1], k[2] + 0.06), pitch=LEV_PITCH, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((k[0], k[1], k[2] + 0.002), pitch=LEV_PITCH, duration=1.2)
        yield from L.grip(FINGER_CLOSED, duration=0.6)

    yield from ctx.both(right_take_cup(), left_take_lever())
    log("cup under the nozzle, knob in hand")
    # pull the lever down along its arc
    n = 30
    for i in range(1, n + 1):
        th = PULL_DEG * i / n
        yield from L.move(knob(th), pitch=LEV_PITCH + 0.5 * th, duration=1.5 / n)
    log("lever down")

    # swirl: the cup's centre circles about the nozzle at a fixed radius and rate (three turns over the serving) and is
    # lowered at a fixed rate (open loop, but on the cup's measured pose: it may sit off centre in the grip)
    t0, t_last, t_cut = ctx.t, 0.0, None
    tcp_cmd = np.array([X_N, Y_N, hold_z])
    off = None
    while t_cut is None:
        if m.get("feed", 0) >= FEED_END - 1e-9 and m.get("travel", 0) >= FEED_END - 0.005:
            t_cut = ctx.t  # the serving is all out: the cup is full
        el = (t_cut if t_cut is not None else ctx.t) - t0
        phi = SWIRL_W * el
        t = _cup_T(ctx.states())
        want = np.array([X_N + SWIRL_R * math.cos(phi), Y_N + SWIRL_R * math.sin(phi),
                         NOZ_TIP - HOLD_RIM_GAP - CUP_H - LOWER_RATE * el])
        if off is None:  # the cup's base centre relative to the TCP, measured once (the cup may sit off centre)
            off = t[:3, 3] - tcp_cmd
        tcp_cmd = want - off
        yield from Rt.move(tuple(tcp_cmd), pitch=CUP_PITCH, duration=1.0 / CONTROL_HZ)
        if ctx.t - t_last > 2.0:
            t_last = ctx.t
            log(f"swirling turns={phi / (2 * math.pi):.2f} cup_err={np.round((want - t[:3, 3]) * 1000)}mm")
        if ctx.t > 90:
            break
    log("serving out: letting the lever up")
    # let the lever up along its arc (the valve closes below 12 deg and the stream breaks off), holding the cup still
    for i in range(1, 9):
        yield from L.move(knob(PULL_DEG * (1 - i / 8)), pitch=LEV_PITCH + 0.5 * PULL_DEG * (1 - i / 8), duration=0.05)
    yield from L.grip(FINGER_OPEN, duration=0.4)
    yield from Rt.wait(HOLD_AFTER_S)
    log("stream cut")
    # settle, let the lever up, lower the cup clear of the nozzle, draw it out from under the head and set it down on
    # the mat; every target is for the cup's base centre, offset to the TCP as the cup sits in the grip
    cup0 = _cup_T(ctx.states())[:3, 3]
    off = tcp_cmd - cup0  # now TCP minus cup

    def cup_to(p, duration):
        yield from Rt.move(tuple(np.asarray(p) + off), pitch=CUP_PITCH, duration=duration)

    k = knob(0.0)
    yield from L.move((k[0] - 0.08, k[1], k[2] + 0.08), pitch=LEV_PITCH, duration=1.0)
    low = cup0 - (0.0, 0.0, 0.06)
    yield from cup_to(low, 1.5)
    yield from cup_to(low - (0.09, 0.0, 0.0), 1.5)
    yield from cup_to((MAT_XY[0], MAT_XY[1], TOP + 0.03), 2.0)
    yield from cup_to((MAT_XY[0], MAT_XY[1], TOP + 0.002), 1.2)
    yield from Rt.wait(0.5)
    # let go gently: ease the squeeze to light contact, then open (opening at once from a firm grip flings the cup)
    yield from Rt.grip(CUP_FINGER - 0.008, duration=1.0)
    yield from Rt.wait(0.5)
    yield from Rt.grip(FINGER_OPEN, duration=1.0)
    # back out the way the hand came in (the open fingers straddle the cup below its wider rim, so lifting straight up
    # can catch the rim), then up
    g = np.array(mk.tcps()[1], dtype=float)
    yield from Rt.move(tuple(g - (0.10, 0.0, 0.0)), pitch=CUP_PITCH, duration=1.5)
    yield from Rt.move(tuple(g - (0.10, 0.0, -0.08)), pitch=CUP_PITCH, duration=1.0)
    log("cup on the mat")
    yield from Rt.wait(2.0)
    log("end")
