# ros2_nav_stack

## Packages

All wheelybot packages are related to a differential drive robot intended for testing in simulation.

* [slam_2d](slam_2d): A 2D grid-based FastSLAM implementation
* [wheelybot_control](wheelybot_control): Controllers for wheelybot
* [wheelybot_description](wheelybot_description): URDF robot description for wheelybot
* [wheelybot_sim](wheelybot_sim): Gazebo integration for wheelybot

## Dependencies

* Eigen

```sh
sudo apt install libeigen3-dev
```

## Tests

Any tests can be run according to [ROS2 documentation](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Testing/CLI.html).

```sh
colcon test --ctest-args tests
colcon test-result --all --verbose
```
