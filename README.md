# revD_lidar

The Rev D W11.25 mecanum base with an LDRobot D500 LiDAR, four VL53L7CX ToF sensors and a BNO085 IMU, in ROS 2 Jazzy and Gazebo Harmonic. Mapping, localization and navigation come first, then small learned models.
Sibling of [revD_ardu](https://github.com/Safwanny/revD_ardu) (the same base with a ToF camera turret), but independent of it.

Status: in simulation, the robot twin (model, Gazebo, teleop, sensors, EKF odometry), the safety chain, slam_toolbox mapping, AMCL localization and Nav2 to goals on the saved map.

## Layout

| Path | What |
| --- | --- |
| `revd_lidar_description/` | URDF/xacro, meshes imported from the CAD, RViz configs |
| `revd_lidar_gazebo/` | Gazebo worlds |
| `revd_lidar_bringup/` | Simulation launch, controller, EKF and safety-chain (twist_mux) configuration |
| `revd_lidar_navigation/` | Mapping (slam_toolbox), localization (AMCL or slam_toolbox) and Nav2 launches and configuration, run beside the simulation or the robot |
| `revd_lidar_tools/` | Sensor models (`lidar_sim`, `tof_sim`, `tof_rows`, `imu_bias`), evaluation (`odom_eval`, `slam_eval`, `map_score`, `loc_score`, `nav_eval`), `stop_simulation` |
| `revd_lidar_safety/` | C++ safety governor: limits every command by the ToF ring and the LiDAR |
| `ml/` | Python project (uv) for datasets, training and evaluation of small models |
| `data/` | Rosbags and datasets, git-ignored (see `data/README.md`) |

Commands: [GUIDE.md](GUIDE.md).
