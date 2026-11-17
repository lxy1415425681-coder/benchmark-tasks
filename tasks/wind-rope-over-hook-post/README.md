# Wind a rope three times over a hook post, holding both ends clear of the table

## Run a Dream-1 trial
```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Wind the rope over the hook's post: hold one end of the rope in the air with one hand, to the left of the hook. With the other hand, carry the rope round the front of the post and up over the post's top to its right side, three times, taking hold of the rope further along after each pass. End with both hands holding the rope and no part of the rope touching the table." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/coil-rope-dream
```

## Run the scripted demo
The demo is closed-loop (`demo_script(ctx)` in `task.py`, reading the live rope state) and runs through
`cabledemo.py`, which writes `top_cam.mp4`, `left_cam.mp4`, `right_cam.mp4`, `setup.png` and `result.json`
(the `task_score`) to the output folder:
```bash
python cabledemo.py task.py ~/yam-eval/coil-rope-demo --max-steps 2000
```
The left hand takes the rope 3 cm from its left end and holds it 22 cm to the left of the post, 10 cm above the
table. The right hand makes three passes. In each pass it carries the rope round the front of the post, about 16 cm
out, then comes in close on the post's left side. It goes up and over the top, 20 cm above the table, to the right
side, and then out until the rope from the post top is taut. It takes the rope 35 cm from the left end for the first
pass. After the first and second passes it lets go and takes the rope again at 58 cm and then 82 cm. After the third
pass it keeps hold. The left hand then rises 1 cm at a time until no part of the rope touches the table (2 cm was
enough). The demo reaches 1.0 after 1,295 steps (about 65 s): 3 passes, both hands holding, the rope clear of the
table.

## Make videos of the Dream-1 trial
```bash
for log in ~/yam-eval/coil-rope-dream/*.json; do inspect-robots video "$log"; done
```

## Files
- `task.py`: the yam_arena task file (hook, rope, scoring and the scripted demo). `score()` counts the passes over
  the post top and must be called once per control step, as the `task_score` scorer does.
- `cablekit.py`: the rope simulation (jointed capsules, with flat-sided colliders so the rope cannot roll on the
  table), copied unchanged from `tasks/figure-eight-cable-on-cleat` in this repository.
- `cabledemo.py`: the closed-loop demo runner, copied unchanged from `tasks/hang-cable-between-hooks`.

## Runtime and seeds
2,000 steps (100 seconds at 20 Hz) per trial. There are no seeds: the starting layout is fixed (`object_jitter=0`).
