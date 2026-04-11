# Drunky Bimanual Bartender Bot

Two SO-101 arms controlled via LeRobot that mix drinks from text commands. A user types a drink name (e.g., "rum and coke"), and the system identifies bottles via wrist-mounted camera, pours ingredients into a center cup, and stirs.

## How It Works

1. **Text command** → parsed into a sequence of ingredients
2. **Vision (YOLOv8)** → identifies and localizes shot bottles on the workspace
3. **ACT policy** → each arm independently picks up a bottle, pours into the center cup, and disposes
4. **Bimanual stirring** → one arm holds the cup, the other stirs

## Hardware

- 2× SO-101 arms (leader + follower pairs)
- Wrist-mounted camera (Intel RealSense D435)
- Stationary workspace with center cup and 4–5 shot bottles
- NVIDIA RTX 5090 for training and inference



## Timeline

| Weeks | Milestone |
|-------|-----------|
| 1–3 | Calibrate arms, collect 50 demos, train single-arm pour policy |
| 4–7 | Second arm, vision integration, multi-ingredient sequencing, bimanual stirring |
| 8–11 | Text interface, integration testing, quantitative evaluation, final demo |

## Evaluation

- **Pour success**: >70% of pours land in cup without spillage (over 10 trials)
- **Drink completion**: >70% of drinks made without spills or missed ingredients
- **Cycle time**: seconds from command to finished drink
- **Generalization** (stretch): handle unseen bottles given a label

## References

- [LeRobot](https://github.com/huggingface/lerobot) — policy training and robot control
- [ACT (Action Chunking with Transformers)](https://tonyzhaozh.github.io/aloha/) — bimanual manipulation policy architecture
- [YOLOv8](https://github.com/ultralytics/ultralytics) — bottle detection
