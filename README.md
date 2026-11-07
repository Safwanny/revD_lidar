# revD_lidar

LiDAR experiments and small neural-network models for the Rev D robot, in ROS 2 Jazzy.
Sibling of [revD_ardu](https://github.com/Safwanny/revD_ardu), which holds the robot model and simulation.

Status: project skeleton only. Sensor, driver and models are not chosen yet.

## Layout

| Path | What |
| --- | --- |
| `ml/` | Python project (uv) for datasets, training and evaluation of small models |
| `data/` | Rosbags and datasets, git-ignored (see `data/README.md`) |

ROS 2 packages (`lidar_*`) are added at the repo root as tickets need them.
