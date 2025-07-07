# Kompass Simulation Suite

**kompass-sim** provides ready-to-use simulation and testing environments for 2D robot navigation, fully integrated with [Kompass](https://github.com/automatika-robotics/kompass) stack. This repository is created for rapid testing, visualization, and validation of autonomous navigation algorithms.

## 📦 Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/automatika-robotics/kompass-sim.git
```

### 2. Install Dependencies

- If you're using `rosdep` for the first time:

```bash
sudo rosdep init
rosdep update
```

- Then install the required dependencies:

```bash
rosdep install --from-paths src -y --ignore-src
```

### 3. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```


## Running Simulations

Launch a simulation environment (e.g., TurtleBot3 in Webots):

```bash
ros2 launch kompass_sim webots_turtlebot3.launch.py
```

This brings up the robot in a Webots, ready to be controlled by Kompass (see KOmpass [quick start](https://automatika-robotics.github.io/kompass/tutorials/quick_start.html) instructions).


## 📚 Related Projects

- [kompass](https://github.com/automatika-robotics/kompass) – Event-driven, eay-to-use and GPU powered navigation stack
- [kompass-core](https://github.com/automatika-robotics/kompass-core) – Core motion planning and control library


## Copyright

The code in this distribution is Copyright (c) 2025 Automatika Robotics unless explicitly indicated otherwise.

Kompass is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.


## Contributing

We welcome issues, feature requests, and PRs. Feel free to fork and improve!
