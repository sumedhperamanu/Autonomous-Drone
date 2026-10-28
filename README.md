# Autonomous-Drone

Versions: see docs/STACK.md.

## Get started
1. Open this folder in VS Code and choose "Reopen in Container".
2. In the container terminal:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

## How we work
- Never push to main. Make a branch: p1/..., p2/... or p3/... plus a short topic, e.g. p2/offboard-node.
- One topic per pull request. Open it early.
- Reviewers: P1 for PX4 topics or params/, P3 for frames or sensor topics, P2 for everything else.