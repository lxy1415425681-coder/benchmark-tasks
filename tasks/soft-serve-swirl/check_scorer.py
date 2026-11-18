"""Offline checks of the scorer on hand-built states (no Isaac Sim): python check_scorer.py"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import mechkit as mk  # noqa: E402
import task as T  # noqa: E402
from cablekit import CableState  # noqa: E402

SEG = T.SERVE.seg_len
N = T.SERVE.segments


def serve_state(pts):
    pts = np.asarray(pts, float)
    # resample to N segment centres spaced SEG apart along the given curve, counted back from its end (the tip, which
    # left the nozzle first); a longer curve loses its top
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    s = arc[-1] - (N - 0.5 - np.arange(N)) * SEG
    c = np.stack([np.interp(s, arc, pts[:, k]) for k in range(3)], axis=1)
    sd, ed = c[0] - c[1], c[-1] - c[-2]
    z = np.zeros(3)
    return CableState(pos=c.mean(0), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=c.min(0),
                      high=c.max(0), points=c, start_plug=None, end_plug=None, start_dir=sd / np.linalg.norm(sd),
                      end_dir=ed / np.linalg.norm(ed), seg_len=SEG)


def _helix(cx, cy, z0, turns, r0, r1, rise):
    t = np.linspace(0, 1, 4000)
    r = r0 + (r1 - r0) * t
    a = 2 * math.pi * turns * t
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a), z0 + rise * t], axis=1)


def helix(cx, cy, z0, r0, r1, rise, length=T.SERVE_LEN + 0.01):
    """A rising spiral of the given radii and rise, with as many turns as make it ``length`` long."""
    lo, hi = 0.0, 50.0
    for _ in range(60):
        mid = (lo + hi) / 2
        h = _helix(cx, cy, z0, mid, r0, r1, rise)
        if np.linalg.norm(np.diff(h, axis=0), axis=1).sum() < length:
            lo = mid
        else:
            hi = mid
    return _helix(cx, cy, z0, hi, r0, r1, rise)


def states(serve, cup_xy=T.MAT_XY, cup_z=T.TOP, tilt_deg=0.0, lever=0.0):
    q = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(tilt_deg)))
    st = T.kit.fake_states(q={"lever_hinge": -lever}, deg=True,
                           moved={"cup": ((cup_xy[0], cup_xy[1], cup_z), tuple(q))})
    st["serve"] = serve_state(serve)
    st["mat"] = None
    return st


def run(name, serve, expect, cut=True, flow_on_table=False, overrun=False, **kw):
    mk.new_episode()
    mk.MEMORY.update(feed=T.FEED_END, cut=cut, flow_on_table=flow_on_table, overrun=overrun)
    st = states(serve, **kw)
    got = T._base_score(st)
    print(f"{name:55s} rings={T.rings(st):4.2f} h={T.height_above_rim(st) * 100:5.1f}cm -> {got} (expect {expect})")
    assert abs(got - expect) < 1e-9, name


cx, cy = T.MAT_XY
B = T.TOP + T.WALL + T.SERVE_R
swirl = helix(cx, cy, B, 0.021, 0.021, 0.038)[::-1]  # three rings stacked to the rim; the tip (end of the list) at the bottom
run("neat swirl filling the cup to the rim, on the mat", swirl, 1.0)
run("same, cup not on the mat", swirl - (0.10, 0, 0), 0.75, cup_xy=(cx - 0.10, cy))
run("same, lever still down", swirl, 0.75, lever=25)
zig = np.array([[cx + 0.014 * (-1) ** k, cy + 0.004 * (k % 3 - 1), B + 0.0035 * k] for k in range(26)])
run("folded heap (back and forth), no rings", zig, 0.25)
# a folded heap at the bottom of the cup, then a given number of rings on top (the same total length)
def rings_on_top(turns):
    h = _helix(cx, cy, B + 0.02, turns, 0.021, 0.021, 0.0125 * turns)
    rest = T.SERVE_LEN - np.linalg.norm(np.diff(h, axis=0), axis=1).sum() + 0.005
    k = int(rest / 0.03) + 1
    half = 0.5 * rest / k  # back and forth across the cup, k folds of exactly the remaining length
    fold = np.array([[cx + half * (-1) ** j, cy, B + 0.02 * j / k] for j in range(k + 1)])
    return np.vstack([fold, h])[::-1]
run("folded heap, 1 ring on top", rings_on_top(1.0), 0.25)
run("folded heap, 2 rings on top", rings_on_top(2.0), 0.5)
low = helix(cx, cy, B, 0.021, 0.021, 0.018)[::-1]
run("three rings but 2 cm short of the rim", low, 0.5)
spill = swirl.copy()
spill[-600:, 0] += 0.08
run("swirl with its first part outside the cup", spill, 0.0)
run("knocked over", swirl, 0.0, tilt_deg=40)
run("serve flowed while the cup stood on the table", swirl, 0.0, flow_on_table=True)
run("lever held down 1.5 s past the serving (overfilled)", swirl, 0.0, overrun=True)
run("serving not finished (stream not cut)", swirl, 0.1, cut=False)
on_table = swirl.copy()
on_table = np.vstack([on_table, np.linspace((cx + 0.04, cy, T.TOP + 0.03), (cx + 0.12, cy, T.TOP + T.SERVE_R), 50)])
run("its first part over the rim onto the table (stream not cut)", on_table, 0.0, cut=False)
print("all scorer checks passed")
