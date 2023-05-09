# wheelybot_sim

A package for simulating wheelybot in Gazebo.

## Launch

The launch file spawns wheelybot in a gazebo simulation with controls enabled.

```sh
ros2 launch wheelybot_sim simulate.launch.py 
```

Twist commands can be sent to wheelybot via the `teleop_twist_keyboard` ros2 node.

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```
