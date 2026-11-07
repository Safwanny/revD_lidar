# revD_lidar

The Rev D W11.25 mecanum base with an LDRobot D500 LiDAR, four VL53L7CX ToF sensors and a BNO085 IMU, in ROS 2 Jazzy and Gazebo Harmonic. Mapping, localization and navigation come first, then small learned models.
Sibling of [revD_ardu](https://github.com/Safwanny/revD_ardu) (the same base with a ToF camera turret), but independent of it.

Status: robot twin in simulation (model, Gazebo, teleop, sensors, EKF odometry).

## Layout

| Path | What |
| --- | --- |
| `revd_lidar_description/` | URDF/xacro, meshes imported from the CAD, RViz configs |
| `revd_lidar_gazebo/` | Gazebo worlds |
| `revd_lidar_bringup/` | Simulation launch, controller and EKF configuration |
| `revd_lidar_tools/` | Sensor models (`lidar_sim`, `tof_sim`, `tof_rows`, `imu_bias`), `odom_eval`, `stop_simulation` |
| `ml/` | Python project (uv) for datasets, training and evaluation of small models |
| `data/` | Rosbags and datasets, git-ignored (see `data/README.md`) |

Commands: [GUIDE.md](GUIDE.md).
