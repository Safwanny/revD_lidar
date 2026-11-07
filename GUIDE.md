# GUIDE — building, running and testing

Every command lives here.

## ML environment

```bash
cd ~/ros2_ws/src/revD_lidar/ml
uv sync                 # CPU-only torch, ~200 MB
uv run pytest
```

## ROS 2 build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <pkg> --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --executor sequential --packages-select <pkg> && colcon test-result --verbose
```

## Robot description (`revd_lidar_description`)

Re-import the meshes from the CAD (only after a CAD change; the script checks the wheel numbers and fails loudly):

```bash
cd ~/ros2_ws/src/revD_lidar/revd_lidar_description
python3 scripts/import_meshes.py /media/safwan/Shared/3D_CAD/mecanum_build/builds/Rev_D_W11.25 meshes
```

View the model in RViz (wheel sliders on; `use_joint_state_gui:=false` holds them at zero):

```bash
ros2 launch revd_lidar_description display.launch.py
```

## Simulation (`revd_lidar_bringup`)

Build the five packages:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select revd_lidar_description revd_lidar_gazebo revd_lidar_tools revd_lidar_safety revd_lidar_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Start Gazebo and RViz (worlds: `navigation_basic` (default), `my_world`, `failsafe_course`; `gui:=false` runs Gazebo headless, `seed:=<n>` changes the noise draw):

```bash
source ~/ros2_ws/install/setup.bash && ros2 launch revd_lidar_bringup simulation.launch.py world:=my_world
```

Drive it from a second terminal (`i` forward, `,` back, `j`/`l` turn; hold Shift for strafing: `J`/`L` sideways, `U`/`O`/`M`/`>` diagonals; `k` stops):

```bash
source ~/ros2_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/cmd_vel_teleop
```

Every command goes through the safety chain (`revd_lidar_bringup/config/safety.yaml`): `/cmd_vel_teleop` (priority 100), `/cmd_vel_nav` (10) and `/cmd_vel_policy` (5) → `twist_mux` → `/cmd_vel_mux` → `safety_governor` → `/mobile_base_controller/reference`. The governor (`revd_lidar_safety`, C++) slows only motion that closes on a ToF or LiDAR point, so the robot stops about 3 cm short of a wall. A silent LiDAR stops it until it is back or the governor is in tof_only mode. The controller stops the robot 0.5 s after the last command.

Lock and release the e-stop (it holds until released):

```bash
source ~/ros2_ws/install/setup.bash && ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

```bash
source ~/ros2_ws/install/setup.bash && ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
```

Switch the governor to tof_only mode (no LiDAR; axis moves and clear standing spins only), and back with `data: true`. `/lidar/ok` is latched, so the publisher must be too:

```bash
source ~/ros2_ws/install/setup.bash && ros2 topic pub --once --qos-durability transient_local /lidar/ok std_msgs/msg/Bool "{data: false}"
```

Check the sensors (`/scan` 500 beams at 10 Hz in `laser_link`, `/tof/<face>/points`, `/tof/<face>/rows`, `/imu/data`, `/odometry/filtered`):

```bash
ros2 topic hz /scan
```

Stop everything, including Gazebo servers that outlive Ctrl+C (it matches every ROS 2 and Gazebo process on the machine, revD_ardu's included):

```bash
ros2 run revd_lidar_tools stop_simulation
```

## Odometry evaluation (`odom_eval`)

Start the simulation with room for a 2 m square, headless:

```bash
ros2 launch revd_lidar_bringup simulation.launch.py gui:=false rviz:=false x:=-4.0 y:=-3.2
```

Run the evaluation in a second terminal. It drives a 60 s standstill, the square, the strafed square and a calibration turn (about 7 min sim time), writes TUM files and `report.txt`, and exits 0 only on PASS:

```bash
ros2 run revd_lidar_tools odom_eval --ros-args -p use_sim_time:=true -p output_dir:=$HOME/ros2_ws/src/revD_lidar/data/odom_eval
```

Rotation calibration: the report's `turn: wheel / true` line is the factor for `kinematics.sum_of_robot_center_projection_on_X_Y_axis` in `revd_lidar_bringup/config/controllers.yaml`. Multiply, rebuild, rerun until it reads 1.000 ± 0.005, and record the value and run there.

Plot a run:

```bash
ros2 run revd_lidar_tools odom_eval_plot $HOME/ros2_ws/src/revD_lidar/data/odom_eval
```

## Tests

Unit tests (no simulator):

```bash
colcon test --executor sequential --packages-select revd_lidar_description revd_lidar_gazebo revd_lidar_tools revd_lidar_safety && colcon test-result --verbose
```

Bringup: the wheel-contact check and the `test_sensor_topics` and `test_safety_chain` launch tests (each starts Gazebo headless, about 5 min in all, needs the GPU for ogre2):

```bash
colcon test --executor sequential --packages-select revd_lidar_bringup && colcon test-result --test-result-base build/revd_lidar_bringup --verbose
```
