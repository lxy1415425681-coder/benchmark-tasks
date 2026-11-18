# Serve a soft-serve swirl: pull the lever with one hand, circle and lower the cup under the nozzle with the other

## Run a Dream-1 trial
```bash
inspect-robots run --policy dream --policy-checkpoint dreamlabs/dream-1-beta --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Serve a soft-serve swirl in the sundae cup: hold the cup under the machine's nozzle with one hand and pull the lever down with the other. While it flows, move the cup in circles under the nozzle and lower it slowly, so the serve is laid in rings stacked up to the cup's rim. When the cup is full, let the lever up, then set the cup down upright on the green mat." \
  --scorer task_score --max-steps 1200 --store-frames --no-prompt --log-dir ~/yam-eval/soft-serve-dream
```

## Make videos of the Dream-1 trial
```bash
for log in ~/yam-eval/soft-serve-dream/*.json; do inspect-robots video "$log"; done
```

## Run the scripted demo
The demo is closed-loop (`demo_script(ctx)` in `task.py`, reading the live cup and lever state) and runs through
`servedemo.py`, which writes `top_cam.mp4`, `left_cam.mp4`, `right_cam.mp4`, `setup.png` and `result.json`
(the `task_score`) to the output folder:
```bash
python servedemo.py task.py ~/yam-eval/soft-serve-demo --max-steps 1200
```
The right hand takes the cup from the side, 2 cm under its rim, and checks that the cup sits centred between the
fingers (it re-grips if not). It holds the cup with the rim 5 mm under the nozzle. The left hand takes the lever's
knob and pulls it down along its arc to the stop. While the serve flows, the right hand moves the cup in 2.1 cm
circles about the nozzle at a fixed rate, three turns over the 8 s serving, and lowers it steadily, 1.6 cm in all.
When the serving is all out, the left hand lets the lever up and the stream breaks off. The right hand lowers the cup, draws it out from
under the machine's head and sets it on the mat. It eases its grip, opens and backs away sideways. The demo reaches
1.0 after 653 steps (about 33 s): 2.7 rings, the top of the serve 1.9 cm above the rim, the cup upright on the mat.

## Files
- `task.py`: the yam_arena task file. It defines the machine, lever, feed, cup, serve and mat, the machine's self-closing
  valve and the serve's stickiness (per-step hooks), the scorer and the scripted demo. `score()` must be called once
  per control step, as the `task_score` scorer does. The serve is drawn as one smooth tube through its centre line;
  the physics uses the capsule chain underneath.
- `mechkit.py`: rigid parts, hinges and sliders for yam_arena task files (the machine, lever, feed piston and cup).
  It is the `mechkit.py` used by other tasks in this repository, with joint friction added.
- `cablekit.py`: the jointed-capsule rope used for the serve. It is adapted from `tasks/figure-eight-cable-on-cleat`:
  it adds a hanging start (`pitch`), a weld of the cable's upper end to another body (the feed piston) and
  sphere colliders for short segments.
- `servedemo.py`: the closed-loop demo runner (both arms at once, reading live object states).
- `check_scorer.py`: offline checks of the scorer on hand-built serve shapes (`python check_scorer.py`, no simulator).

## Runtime and seeds
1,200 steps (60 seconds at 20 Hz) per trial. There are no seeds: the starting layout is fixed (`object_jitter=0`).
A trial takes about 8 minutes of wall-clock time on one RTX 5080.
