# barn_ddr_bridge

Bridge to run **DDR-opt** navigation stack on the **BARN** benchmark.

## What it does

- **LaserScan → PointCloud2**: Converts BARN Gazebo `/front/scan` to `/laser_simulator/local_pointcloud` for DDR-opt's SDF map.
- **CarState → Twist**: Subscribes to DDR-opt MPC output (`/ddr_cmd`) and publishes `geometry_msgs/Twist` to `/cmd_vel` for Jackal.
- **TF**: Publishes static `odom` → `world` so DDR-opt's "world" frame exists (identity).

## Prerequisites

1. **Single catkin workspace** containing both BARN and DDR-opt packages:
   - BARN: `jackal`, `jackal_simulator`, `jackal_desktop`, `jackal_helper`, `eband_local_planner`, and this repo (pipeline with `barn_ddr_bridge`).
   - DDR-opt: `plan_manager`, `mpc_controller`, `front_end`, `back_end`, `plan_env`, `carstatemsgs` (and any other DDR-opt dependencies).

2. **ROS** (Melodic or Noetic), **laser_geometry**, **tf2_ros**, **tf2_sensor_msgs**:
   ```bash
   sudo apt install ros-${ROS_DISTRO}-laser-geometry ros-${ROS_DISTRO}-tf2-sensor-msgs
   ```

## Build

In your workspace (with both BARN and DDR-opt packages in `src/`):

```bash
cd /path/to/your_ws
catkin_make   # or: catkin build
source devel/setup.bash
```

## Run DDR-opt on BARN

From the pipeline directory (or with workspace sourced):

```bash
# Single run, world 0, with DDR-opt
python3 run.py --world_idx 0 --nav_stack ddr_opt

# With GUI
python3 run.py --world_idx 0 --nav_stack ddr_opt --gui
```

Default navigation stack remains `dwa` (move_base DWA) for backward compatibility.
