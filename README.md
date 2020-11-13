# panda_grasp_server

Simple package to perform basic actions with the robot (i.e. move to a pose, grasp given a target pose, home) within a workbench + table collision avoidance scene. The python node exposes a bunch of services under its namespace.

Uses messages and services defined in `hsp-panda/panda_ros_common`.

Launch with

`roslaunch panda_grasp_server grasp_server.launch start_realsense:=false enable_force_grasp:=false`

if using soft fingers, with

`roslaunch panda_grasp_server grasp_server.launch start_realsense:=false enable_force_grasp:=true`

if using hard fingers.