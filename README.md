# ENG612 Project 2: Wall Following with TurtleBot3 Burger

ROS 2 Humble package (`wall_follower`) that keeps a TurtleBot3 Burger 0.30 m from a wall
using PID control on the laser scan. Tested in Gazebo and on the real robot.

## Layout
- `src/wall_follower/` ROS 2 package (nodes, launch file, world)
- `scripts/analyze_runs.py` run comparison and plots
- `docs/logs/` recorded runs (CSV)

## Build
    cd ~/ENG612-Project-2
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Run in simulation
    export TURTLEBOT3_MODEL=burger
    ros2 launch wall_follower sim.launch.py x_pose:=0.5 y_pose:=0.3
    ros2 run wall_follower wall_follower_node --ros-args -p mode:=extended -p use_sim_time:=true \
      -p v_max:=0.08 -p v_min:=0.02 -p kp_w:=3.0 -p kd_w:=0.8 -p lookahead:=0.2 -p front_stop:=0.34
