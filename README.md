# 2D Navigation Simulations and Tests
This package provides launch and configuration files for 2D robot navigation in simulation

## Install from source

- Clone the repository into your ROS2 workspace

```shell
cd my_ros2_ws/src
git clone https://github.com/automatika-robotics/kompass-sim.git
```

- Install the dependencies with `rosdep`

If this is the first time using `rosdep`, it must be initialized via:

```shell
sudo rosdep init
rosdep update
```
then run:

```shell
rosdep install --from-paths src -y --ignore-src
```

- Build and source your workspace

```shell
colcon build
source install/setup.bash
```

- Now you can launch any of the available simulation directly using `ros2` command line:

```shell
ros2 launch kompass_sim webots_turtlebot3.launch.py
```
