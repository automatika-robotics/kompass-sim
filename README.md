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

## 🐳 Running with Docker

If you don't have ROS 2 installed, you can run the simulation from a prebuilt
container instead. The image is based on ROS 2 **Jazzy**, contains this package
and all of its dependencies, and **already ships Webots inside it**. The default command runs the quick-start launch file above (`webots_turtlebot3.launch.py`).

The container is meant to be run with:

- `--network host`, so a Kompass/EMOS stack running **outside** the container
  (on the host or another machine) can discover and talk to the simulation over
  DDS, and
- access to the host's X server and GPU, so the Webots and RViz windows show up
  on your screen.

### 1. Get the image

Pull the prebuilt image from Automatika's registry:

```bash
docker pull automatika/kompass-sim:jazzy
```

…or build it locally from the repository root:

```bash
docker build -t automatika/kompass-sim:jazzy .
```

### 2. Allow the container to use your display

Run this once per login session on the host:

```bash
xhost +local:root
```

### 3. Run the simulation

```bash
docker run -it --rm \
  --network host \
  --ipc host \
  --env DISPLAY="$DISPLAY" \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  automatika/kompass-sim:jazzy
```

This starts Webots + the robot driver + localization + RViz, exactly like the
native quick start. Kompass running on the host can now be pointed at the
simulation.

**Notes**

- **NVIDIA GPUs:** install the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  and add `--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all` to the `docker run`
  command (you can drop `--device /dev/dri` in that case).
- **Talking to Kompass/EMOS:** with `--network host` discovery happens on
  `localhost`. Make sure both sides share the same `ROS_DOMAIN_ID` (add
  `--env ROS_DOMAIN_ID=<id>` if you set one on the host) and the same
  `RMW_IMPLEMENTATION`.
- **Simpler privileges:** if the X11/device flags above are inconvenient, adding
  `--privileged` is a coarser alternative that also gives the container display
  access.
- **Run something else:** append any command to override the default launch, e.g.
  `docker run ... automatika/kompass-sim:jazzy bash` for a shell, or
  `... ros2 launch kompass_sim webots_turtlebot3_rgbd.launch.py`.

## 📚 Related Projects

- [kompass](https://github.com/automatika-robotics/kompass) – Event-driven, eay-to-use and GPU powered navigation stack
- [kompass-core](https://github.com/automatika-robotics/kompass-core) – Core motion planning and control library

## Copyright

The code in this distribution is Copyright (c) 2025 Automatika Robotics unless explicitly indicated otherwise.

Kompass is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.

## Contributing

We welcome issues, feature requests, and PRs. Feel free to fork and improve!
