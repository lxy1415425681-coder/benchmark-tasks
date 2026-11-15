# Coil and hang a rope on a wall hook

## Run a Dream-1 trial
```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Coil the rope onto the wall hook: hold one end of the rope with one hand, and use the other hand to wind the rope in loops around the hook, starting from the far end and moving forward, with each loop slightly higher than the last. Then release both hands." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/coil-rope-dream
```

## Run the scripted demo
```bash
inspect-robots run --embodiment yam_arena \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Coil the rope onto the wall hook: hold one end of the rope with one hand, and use the other hand to wind the rope in loops around the hook, starting from the far end and moving forward, with each loop slightly higher than the last. Then release both hands." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/coil-rope-demo \
  --policy yam_scripted
```

## Make videos
```bash
for log in ~/yam-eval/coil-rope-dream/*.json; do inspect-robots video "$log"; done
for log in ~/yam-eval/coil-rope-demo/*.json; do inspect-robots video "$log"; done
```

## Runtime
3000 steps (150 seconds at 20 Hz)
