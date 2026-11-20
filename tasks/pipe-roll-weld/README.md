# pipe-roll-weld

**Instruction:** see `task.json` (also `INSTRUCTION` in `task.py`).

Two 1 1/2 in aluminium pipes lie end to end in steel V-blocks, 3 cm apart. One hand holds the fixed pipe while the
other slides the loose pipe home. Pipe-welding locking pliers, locked on their stand, are unlocked, carried over and
locked onto the joint: their jaws end in two slotted half-ring bands that centre on the joint and hold the pipes in
line. With the pliers on, the joint is tacked through the bands' slots, the pipes (and pliers) turned a little each
way in their V-blocks so each side comes up while the pliers' handles tip away from the gun. The pliers are then
turned back upright, unlocked, and locked back on their stand, and the rest of the joint is welded all round, the
pipes turned in their V-blocks to bring each part on top. The gun goes back on its rest and both hands stand clear.

## Why it matters

Butt-welding pipe in V-blocks with a pair of pipe-welding locking pliers is everyday fabrication work. It takes both
hands at once and in turn: one turns and steadies the pipe while the other keeps the gun at the joint, the pliers'
locking toggle must be worked (squeeze shut to lock; squeeze the release trigger to unlock), and the pliers, heavy and
standing over the pipe, must be turned with it without rolling it over or letting their handles block the gun.

## Real parts

| object | real part | size / mass | pose (cm; x forward, y left, heights above the table) |
|---|---|---|---|
| pipes (2) | 1 1/2 in NPS Schedule 40, 6061-T6 aluminium, one end bevelled 37.5 deg with a 1.6 mm land, wire-brushed | 48.3 mm OD, 3.7 mm wall, 30 cm, 0.42 kg each | axes at x 19, 3.45 up; fixed y 0..+30, loose y -33..-3 |
| V-blocks (2) | steel, 90 deg V with a relief slot | 4 cm long | centred y -26 and +26 |
| end stops (2) | 6 mm steel plates on feet | 4 x 6 cm | y -33.6 and +30.6 |
| welding pliers | JTLD pipe-welding locking pliers for 1 1/2 in pipe (sold on Alibaba; Vise-Grip-style toggle, preset jaw screw), stainless | 0.5 kg, 186 mm; bands 49.9 mm inside, 5 mm thick, 44 mm wide, each wrapping 5-150 deg, slotted 20 mm from the tips to 40 deg from the top; about 20-30 N at the grips to lock or release | locked on their stand at x 19, y +42 |
| pliers stand | 10 cm stub of the same pipe on a steel foot, a 3 mm ridge along its top | | x 19, y +42 |
| MIG gun | MB15AK MIG torch (about $130) with a Teflon liner (needed to feed 1.0 mm aluminium wire), in the Euro torch socket of any budget MIG welder rated for aluminium (about $200-400), 1.0 mm ER4043 wire, cable on an overhead balancer | 0.6 kg; 15 cm handle, 16 mm copper nozzle, wire 10 mm out | on its steel rest at x 10, y -30, handle along y |

The fixed pipe carries painted roll marks 1-3 at 120 deg near its outer end and a scribed ring 12 mm from the joint,
ticked every 30 deg: the 12 segments the score counts.

## Scoring

A segment is one of the joint's 12 30-degree stretches.

- **0** if a pipe leaves its V-block (its axis more than 15 mm from where the block holds it, or its end less than
  1 cm onto the block) or the pliers or the gun fall off the table.
- Otherwise **0.9 x the fraction of the 12 segments welded** (rounded to 0.001),
- **capped at 0.5** if fewer than 3 tacks were laid (below), or if
  the pipes are out of line (axes over 0.8 mm or 1 degree apart).
- **1.0** when all 12 are welded and the pliers are locked back on their stand (centre within 2 cm of where they
  started), the gun is on its rest (handle centre within 2 cm, turned under 20 degrees), both hands (gripper centre
  points) are 6 cm or more from the pipes, pliers and gun, and all of them are still (under 0.01 m/s and
  0.2 rad/s), held for 1 s (20 control steps). Success is the score reaching 1.0.

**Simulation weld rule:** the held gun's wire tip heats the joint where it is within 4 mm of it, both along the pipe
and from its surface, with the pipe ends butted (gap under 2 mm: a bead can't bridge an open gap); each control step
there adds 0.05 s to the 10-degree third of the joint under it. A segment is welded (drawn orange) once each of its
three thirds has had at least 1 s and the segment 3 s in all, so the gun has to travel along it at a welding pace
(a segment is 12.6 mm of joint: about 2.5-4 mm/s), not just touch it. Past 5 s it is burned through (a hole in the
real pipe): it never counts again. With the pliers locked on the joint, each 10-degree third (4.2 mm) that gets its
1 s is a tack; the three tacks may be side by side (one 12.6 mm bead) or apart. Three tacks join the pipes rigidly.

**Real trial:** after the pipe has cooled under 50 C, a segment counts when its bead covers the joint over its whole
length (both edges at least 1.5 mm past the bevels' edges), with no hole through the wall (a lamp inside the bore
shows none), no visible crack and no pore over 1 mm, checked with a weld gauge, a 10x loupe and photographs; tacks are
counted from the video: the arc on for at least 1 s over each of at least three different 10-degree stretches of the
joint (located by the ring's ticks) while the pliers are on; in line is checked with a 300 mm straightedge laid along the
pipe across the joint at four places 90 degrees apart (no gap over 0.8 mm at its ends); the pliers' and gun's places
with a tape measure.

## Real-trial safety

- The supervisor, outside the welding curtains, holds a hold-to-run switch wired to the gun's trigger socket: the
  arc cannot strike without it. It is closed whenever the wire tip is within 1 cm of the joint and open otherwise, so
  it decides only when the arc may strike, never where the gun goes. A separate emergency stop cuts the robots' power
  and the welder's output.
- A fume extractor hood (100 mm or larger) sits within 30 cm of the joint and runs for the whole trial.
- Cameras sit behind auto-darkening welding filters (shade 5-8 only while the arc is struck, clear between passes)
  and cover glass, so the policy sees the scene normally between passes.
- Curtains (EN ISO 25980, at least 1.8 m) close the cell with nobody inside while the arc can strike; aluminised
  glass-fibre arm covers, 1200 C ceramic-fibre finger sleeves, a welding blanket on the table.

## Simulation notes

- **Pliers:** three rigid bodies: the body (fixed jaw and band, fixed handle, screw), the movable jaw and handle (on a
  hinge at the pivot, a spring opening it to 30 deg) and the red release-trigger plate (on a preloaded slider along the
  movable handle, 32 N preload, released past 3 mm, i.e. ~36.5 N). The toggle is modelled on the hinge's spring:
  squeezed shut (within 4 deg), it locks (a stiff spring holds the jaws shut). Once the hand has let the trigger spring
  back out, a squeeze that presses it in past 3 mm unlocks them, and they can't lock again until the jaws have opened
  past 8 deg. The YAM gripper fully closed squeezes ~41 N (measured), enough to release them.
- **Locked over the joint** (within 12 mm and 10 deg of centred), the bands close round both pipes: the pliers are
  centred on the joint and the pipes pulled into line (fixed constraints). Locked anywhere else (on their stand, in the
  air) they hold nothing.
- **Bands:** the fixed jaw's band reaches 90 deg (the pipe's mid-height), so the pliers go on and come off straight up
  and down; the photographed pliers' bands both wrap to ~150 deg. The movable band wraps to 150 deg.
- **Pipes** are rigid, with real contact in the V-blocks (aluminium on steel, friction 0.6); round bodies collide as
  24-sided hulls. The welded pipes creep up to ~3 cm along their axis over a run as they are turned; the end stops
  bound that.
- **Not simulated:** arc, heat, wire feed, bead shape, spatter, distortion, cable drag and the arm covers; the finger
  sleeves are thicker, grippier pads (friction 0.7).

## Run a trial

From this directory, with the yam_arena environment (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1):

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --max-steps 7200 \
    --instruction "$(python -c "import json; print(json.load(open('task.json'))['instruction'])")"
```

## Dream-1 run

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --scorer task_score --max-steps 7200 --store-frames --log-dir dream-run \
    --instruction "$(python -c "import json; print(json.load(open('task.json'))['instruction'])")"
```

Dream-1 scored 0: it ran the full 7,200 steps without welding any segment. Its arms moved about the pipes, the gun and
the pliers' stand, but the pipes' 3 cm gap was never closed, the pliers stayed on their stand and the gun near its rest.

## Scripted demo

```bash
python rundemo.py task.py demo_out --max-steps 7200
```

Closed-loop (it reads the live object states). Left hand holds the fixed pipe while the right slides the loose
pipe home (1.5 mm gap). Left: squeezes the pliers' handles fully (unlocks them on their stand), eases to ~10 N (the
jaws spring open to ~11 deg), lifts them, carries them over the joint and lowers them straight down, correcting its
aim 4.5 cm and 8 mm above the seat; squeezes shut (~28 N): they lock, centred, the pipes pulled into line. Right takes
the gun. Left grips the fixed pipe and turns pipes and pliers 30 deg clockwise (seen from the left: handles tipping
toward the robot) and holds them; right welds along the side that came up through its slot, at about 4 s per
segment (4 tacks); left turns them back upright; the same 30 deg the other way (3 more tacks). Left unlocks the pliers
(full squeeze), lifts them straight off and locks them back on their stand; then turns the pipes five times (bringing
the unwelded stretches on top) while right welds the rest at the same pace, lifting the wire tip clear over stretches
already done so none passes 5 s (segments got 3.2-4.8 s). Right puts the gun back on its rest; both arms clear.
Score 1.0, success at step 6,118 (306 s). `demo.mp4` is the three cameras side by side.

Test variant: `PRW_DEMO=noclamp python rundemo.py task.py out --max-steps 7200` welds the whole joint the same way
but never puts the pliers on: with nothing holding the pipes together, turning them opens the joint past 2 mm after
the first segment, so it welds 1 of 12 with no tacks and scores 0.075 (under the 0.5 cap either way).

## Runtime and seeds

- Run limit 7,200 control steps (6 minutes at 20 Hz); physics 240 Hz.
- Fixed starting poses (`object_jitter=0`), no randomisation; seed 0.
- One trial of the demo takes about 11 minutes of wall time on one RTX 5080 including Isaac Sim start-up.

## Files

`task.py` (scene, pliers mechanism and lock logic, weld rule, scorer, scripted demo), `mechkit.py` (compound rigid
parts, joints, latches and the yam_arena hooks), `rundemo.py` (the closed-loop demo runner: writes the three camera
videos, `setup.png` and `result.json`). No third-party assets. No hardware validation has been performed.
