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
