# Robotic Car Painting Cell : ROS 2 + Gazebo

Robotic car painting cell simulation using ROS 2, MoveIt 2, and Gazebo Harmonic.

| Item | Selection |
| --- | --- |
| Candidate | Aditya Pachauri |
| Software | ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, RViz 2 |
| Robot | Universal Robots UR20, 6 DOF, 1.75 m nominal reach |
| Car | OpenRobotics Hatchback v3, scaled to 4.8 × 1.9 × 1.6 m |
| Demonstrated surfaces | Hood, roof, and one side panel |
| Status | Nine painting strokes planned, collision-checked, and executed in Gazebo |

## What the project demonstrates

- A 12 × 8 m reference-style cell with an 8 × 5 m central bay, 2 × 5 m side bays, upper service rooms, and aligned 2.2 m entry/exit openings through both wall lines.
- The car origin is exactly at the central-bay centre `(0.0, -1.5, 0.0)`; the robot is shifted with it to preserve the validated relative pose.
- A 200 mm spray-nozzle tool attached to the robot flange.
- Three serpentine passes on each of the hood, roof, and side-panel patches.
- Mesh ray casting to locate surfaces, estimate normals, and keep the nozzle TCP 0.25 m from each patch.
- MoveIt 2 inverse kinematics, free-space approach planning, Cartesian painting paths, and collision checking.
- Spray OFF during approach moves and spray ON during painting strokes, with RViz text and a translucent spray cone.
- An intentional intersecting pose that is detected as a car/nozzle collision and is never executed.

This is a software demonstration on selected surface patches. It does not claim full-body paint coverage, paint-film simulation, or production safety certification.

## Verified results

The completed run is recorded in [`results/execution.json`](results/execution.json).

| Metric | Result |
| --- | ---: |
| Painting strokes | 9 |
| Cartesian completion | 1.0 for every stroke |
| Executed approach + painting segments | 18 |
| Planning-scene collision objects | 21 |
| Sampled states checked for collisions | 1,197 |
| Deliberate collision test | Passed; car ↔ spray nozzle detected |
| Deliberate collision executed | No |
| Target TCP standoff | 0.250 m; dense nearest-mesh check 0.246 to 0.250 m |
| Simulated sequence duration | 117.18 s |
| Maximum final joint error | 0.00139 rad |

## Repository structure

```text
.
├── src/painting_cell/             # Simulation, planning, tool, world, and RViz package
├── src/ur_description/            # Minimal upstream UR20 description and licenses
├── scripts/                       # Environment setup scripts (env.sh, env.zsh)
├── results/execution.json         # Machine-readable successful execution evidence
├── media/screenshots/             # Report and submission figures
├── Technical_Report.pdf           # Generated technical report PDF
└── THIRD_PARTY_NOTICES.md         # Asset sources and licenses
```

## Build

The tested environment is Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. Install the ROS dependencies represented by `package.xml`, then build:

```bash
cd /home/aditya-pachauri/Yuktii_AI_Labs_Assignment
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

`PYTHONNOUSERSITE=1` prevents incompatible packages from `~/.local` from overriding the ROS-compatible system Python packages on this machine.

## Run

Terminal 1 : start Gazebo, MoveIt, controllers, and RViz:

```bash
cd /home/aditya-pachauri/Yuktii_AI_Labs_Assignment
# Zsh (your current shell):
source scripts/env.zsh
ros2 launch painting_cell cell.launch.py gui:=true rviz:=true
```

For Bash, use `source scripts/env.sh` instead. Do not source `setup.bash` from Zsh or `setup.zsh` from Bash.

Terminal 2 : validate and execute the sequence:

```bash
cd /home/aditya-pachauri/Yuktii_AI_Labs_Assignment
source scripts/env.zsh
ros2 run painting_cell painting_demo \
  --execute \
  --output results/execution.json \
  --keep-alive
```

`--keep-alive` keeps the car, paths, status, and spray visualization available in RViz after execution. Stop both terminals with `Ctrl+C`.

To plan and validate without moving the simulated robot, omit `--execute` and write to a different result file:

```bash
ros2 run painting_cell painting_demo --output results/validation-only.json
```

## Engineering choice and limitation

The UR20 was selected because its official ROS 2 Jazzy description and Gazebo/MoveIt support made the integration reproducible. In a production automotive paint booth, the design would use a paint-rated, explosion-protected robot and compatible booth equipment, often with a second robot or rail for full-body coverage. The present UR20 setup is an educational simulation, not a production paint-cell recommendation.

See the [technical report](Technical_Report.pdf) for the full implementation explanation and [third-party notices](THIRD_PARTY_NOTICES.md) for asset licenses.
