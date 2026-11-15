"""Coil a rope onto a wall hook by winding it in loops."""
from __future__ import annotations
import math
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np
from cablekit import cable
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = (
    "Coil the rope onto the wall hook: hold one end of the rope with one hand, "
    "and use the other hand to wind the rope in loops around the hook, starting "
    "from the far end and moving forward, with each loop slightly higher than "
    "the last. Then release both hands."
)

HOOK_X = 0.00
HOOK_Y = -0.50
HOOK_Z = TOP + 0.35
HOOK_R = 0.010
HOOK_PROJECTION = 0.05

HOOK_POST = Cylinder("hook_post", pos=(HOOK_X, HOOK_Y, TOP + 0.175), radius=0.012, height=0.35, color=(0.6, 0.6, 0.6))
HOOK_ARM = Cylinder("hook_arm", pos=(HOOK_X, HOOK_Y + HOOK_PROJECTION / 2, HOOK_Z), radius=HOOK_R, height=HOOK_PROJECTION, color=(0.6, 0.6, 0.6))

R = 0.006
ROPE_LEN = 1.20
CORD = cable("rope", start=(0.10, ROPE_LEN / 2, TOP + R + 0.001), yaw=-math.pi / 2, length=ROPE_LEN, radius=R, color=(0.55, 0.35, 0.15), bend_stiffness=0.005, bend_damping=0.001)

OBJECTS = [HOOK_POST, HOOK_ARM, CORD]

TABLE_Z = TOP
HOOK_TIP = np.array([HOOK_X, HOOK_Y + HOOK_PROJECTION, HOOK_Z])
COIL_RADIUS = 0.15
ON_HOOK_DIST = 0.12
HANG_TIME_STEPS = 100
_hang_steps = [0]

def _rope_pts(objects):
    return objects["rope"].polyline()

def off_table(objects):
    return bool(_rope_pts(objects)[:, 2].min() > TABLE_Z + 0.015)

def coiled(objects):
    pts = _rope_pts(objects)
    return bool(np.linalg.norm(pts - pts.mean(axis=0), axis=1).max() < COIL_RADIUS)

def on_hook(objects):
    return bool(np.linalg.norm(_rope_pts(objects).mean(axis=0) - HOOK_TIP) < ON_HOOK_DIST)

def score(objects):
    if on_hook(objects) and off_table(objects):
        _hang_steps[0] += 1
        return 1.0 if _hang_steps[0] >= HANG_TIME_STEPS else 0.75
    if coiled(objects) and off_table(objects):
        return 0.5
    if off_table(objects):
        return 0.25
    return 0.0

def success(objects):
    return score(objects) >= 1.0

def distance(objects):
    return float(np.linalg.norm(_rope_pts(objects).mean(axis=0) - HOOK_TIP))

def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.15
    carry = TOP + 0.30
    NUM_COILS = 4
    COIL_R = 0.06
    # First loop starts farthest from robot (close to wall), each loop moves 2cm forward and 1.5cm higher
    COIL_Y_START = HOOK_Y - 0.05
    COIL_Y_STEP = 0.02
    COIL_Z_START = HOOK_Z - 0.02
    COIL_Z_STEP = 0.015

    pts = ctx.states()["rope"].polyline()
    left_end = pts[0]
    right_end = pts[-1]

    # Left hand grabs one end of the rope
    yield from L.move((left_end[0], left_end[1], hi), finger=FINGER_OPEN, duration=1.5)
    yield from L.move((left_end[0], left_end[1], left_end[2] + 0.01), duration=1.0)
    yield from L.grip(FINGER_CLOSED)

    # Left hand holds rope end fixed near the hook
    yield from L.move((HOOK_X - 0.08, HOOK_Y, HOOK_Z + 0.05), duration=2.0)

    # Right hand grabs the other end of the rope
    yield from Rt.move((right_end[0], right_end[1], hi), finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((right_end[0], right_end[1], right_end[2] + 0.01), duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)

    # Right hand lifts the rope
    yield from Rt.move((right_end[0], right_end[1], carry), duration=1.5)

    # Right hand winds rope around hook: each loop moves forward and higher
    for i in range(NUM_COILS):
        coil_y_center = COIL_Y_START + i * COIL_Y_STEP
        coil_z = COIL_Z_START + i * COIL_Z_STEP
        for s in range(9):
            angle = 2 * math.pi * s / 8
            x = HOOK_X + COIL_R * math.cos(angle)
            y = coil_y_center + COIL_R * math.sin(angle)
            z = coil_z
            yield from Rt.move((x, y, z), duration=0.3)

    # Right hand places rope end onto the hook
    yield from Rt.move((HOOK_X, HOOK_Y + HOOK_PROJECTION + 0.02, HOOK_Z), duration=1.0)

    # Both hands release
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))

    # Both hands move clear
    yield from ctx.both(
        L.move((HOOK_X - 0.15, HOOK_Y, carry), duration=1.0),
        Rt.move((HOOK_X + 0.15, HOOK_Y, carry), duration=1.0),
    )
    yield from L.wait(3.0)
