# ENG612 Project 2: Wall Following with TurtleBot3 Burger

ROS 2 Humble package (`wall_follower`) that keeps a TurtleBot3 Burger 0.30 m from its **left wall**
using PID control on the laser scan. Tested in Gazebo and on the real robot in a 2 m x 2 m box.

**Authors:** Kazi Ragib Yeahsar (GitHub `ragib-yeahsar`) and Mir Tahir Tahsin Ahmad (GitHub `jackutsa`)

## Controllers (set with `-p mode:=...`)
- `baseline`: Normal PID. One PID steers, constant speed.
- `extended`: same steering PID, speed set by simple rules.
- `extended_pid`: Advanced PID. Same steering PID plus a second PID for speed (final design).

## Layout
- `src/wall_follower/` ROS 2 package (nodes, launch file, world)
- `scripts/analyze_runs.py` run comparison and plots
- `docs/logs/` recorded runs (CSV)
- `submission/` report and presentation

## Build
    cd ~/ENG612-Project-2
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Run in simulation (left wall)
Terminal 1:

    export TURTLEBOT3_MODEL=burger
    ros2 launch wall_follower sim.launch.py x_pose:=0.5 y_pose:=1.7

Terminal 2:

    ros2 run wall_follower wall_follower_node --ros-args -p side:=left -p mode:=extended_pid -p use_sim_time:=true \
      -p v_max:=0.08 -p v_min:=0.02 -p kp_w:=3.0 -p kd_w:=0.8 -p lookahead:=0.2 -p front_stop:=0.34

Use `-p mode:=baseline` for the Normal PID and `-p mode:=extended` for the rule-based speed.

## Run on the real robot (left wall)
1. On the robot (Raspberry Pi): `ros2 launch turtlebot3_bringup robot.launch.py`
2. On the laptop:

    ros2 run wall_follower wall_follower_node --ros-args -p side:=left -p mode:=extended_pid -p use_sim_time:=false \
      -p v_max:=0.065 -p v_min:=0.02 -p kp_w:=3.0 -p kd_w:=0.8 -p lookahead:=0.2 -p front_stop:=0.34

3. Place the robot 0.30 m from the left wall, parallel to it. Use `-p v_max:=0.05` as the slower fallback.
4. Stop with Ctrl+C. The robot stops by itself.

## Results (left wall, real robot, laser-measured error)
| Run | RMS (mm) | Worst (cm) | Lap (s) |
| --- | --- | --- | --- |
| Rule-based speed, 0.05 m/s | 4.1 | 5.9 | 158 |
| Advanced PID, 0.05 m/s | 3.8 | 5.4 | 132 |
| Advanced PID, 0.065 m/s | 4.1 | 5.4 | 116 |

See the report in `submission/` for the full results and comparison.
