# Whisky Bot

ROS 2 Jazzy workspace for the Whisky Bot arm, including MoveIt configuration and a
`ros2_control` hardware driver for a PCA9685 PWM board connected to a Raspberry Pi.

The PCA9685 driver controls these open-loop servo joints:

| Joint | PCA9685 channel |
| --- | --- |
| `joint2` | 0 |
| `joint4` | 1 |
| `joint6` | 2 |
| `joint8` | 3 |
| `joint9` | 4 |

It sends PWM commands at 50 Hz by default. Since a PCA9685 cannot measure servo position,
the published joint state is the last command that the driver successfully wrote; it is not
physical feedback.

## Raspberry Pi requirements

Use a 64-bit Ubuntu 24.04 installation on the Raspberry Pi. ROS 2 Jazzy binary packages target
Ubuntu 24.04 and must be installed before building this workspace. A desktop session is required
for the existing full bringup launch because it also starts RViz and MoveIt.

The PCA9685 requires:

- A PCA9685 board connected to the Pi's I2C bus.
- A common ground between the Pi, PCA9685, and servo power supply.
- A dedicated servo power supply sized for the servos and arm load. Do **not** power servos from
  the Raspberry Pi's 5 V rail.
- I2C enabled and access to `/dev/i2c-1`.

### Debian host: run ROS 2 in Docker

If the Raspberry Pi runs Debian instead of Ubuntu, run the ROS 2 setup, build, and launch commands
inside the ROS Jazzy Docker image rather than trying to install the Ubuntu-targeted ROS 2 Jazzy
packages on Debian.

For ROS-only work, start the image used by this project:

```bash
docker run -it --rm --network host ros:jazzy bash
```

For PCA9685 hardware access, the container also needs the host I2C device and the workspace bind
mounted. Run this command from the workspace root on the Raspberry Pi:

```bash
docker run -it --rm \
  --network host \
  --device /dev/i2c-1:/dev/i2c-1 \
  -v "$PWD:/home/ros2_ws" \
  -w /home/ros2_ws \
  ros:jazzy bash
```

Enable I2C on the Debian host before starting the container; a container cannot enable the Pi I2C
controller. The container is ephemeral because of `--rm`, so install the build dependencies inside
it each time unless you create a derived image. Then run the build commands from
`/home/ros2_ws`, after sourcing `/opt/ros/jazzy/setup.bash`. The `--network host` setting allows
ROS 2 discovery between the container and host network. The existing full launch starts RViz, so
use an X/Wayland display setup or omit RViz for headless Raspberry Pi deployments.

## ROS 2 Jazzy setup

Follow the official ROS 2 Jazzy installation instructions for Ubuntu 24.04 first. Install the
ROS 2 apt source, then install the required runtime, build, and I2C packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  i2c-tools \
  libconsole-bridge-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-dev-tools \
  ros-jazzy-ros-base \
  ros-jazzy-controller-manager \
  ros-jazzy-hardware-interface \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-moveit
```

These packages provide the dependencies used by this workspace:

| Package group | Why it is required |
| --- | --- |
| `ros-jazzy-ros-base` | Core ROS 2 runtime, including `rclcpp`, lifecycle support, and plugin discovery. |
| `ros-jazzy-hardware-interface`, `ros-jazzy-ros2-control`, `ros-jazzy-controller-manager` | Provides the `hardware_interface` CMake package, loads the PCA9685 `SystemInterface` plugin, and runs `ros2_control_node`. |
| `ros-jazzy-ros2-controllers` | Provides `joint_trajectory_controller` and `joint_state_broadcaster` declared in `ros2_controllers.yaml`. |
| `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher` | Expands the robot/control description and publishes transforms. |
| `ros-jazzy-moveit`, `ros-jazzy-rviz2` | Required by the current `whisky_bot.launch.xml`, which starts MoveIt and RViz. |
| `ros-dev-tools`, `build-essential`, `python3-colcon-common-extensions`, `libconsole-bridge-dev` | Builds `ament_cmake` C++ packages with `colcon` and supplies the system CMake dependency required by ROS 2 `pluginlib`. |
| `i2c-tools` | Detects and diagnoses the PCA9685 on the I2C bus. |

Initialize rosdep once on a new Pi:

```bash
sudo rosdep init
rosdep update
```

Add ROS 2 to new shell sessions:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Enable and verify I2C

Enable the Raspberry Pi I2C controller, then reboot:

```bash
sudo raspi-config
```

Select **Interface Options**, enable **I2C**, and reboot. If `raspi-config` is unavailable on
your Ubuntu image, enable I2C in the Raspberry Pi boot configuration according to the Ubuntu
Raspberry Pi documentation, then reboot.

After reboot, grant the user access to I2C and verify that bus 1 exists:

```bash
sudo usermod -aG i2c "$USER"
ls -l /dev/i2c-1
i2cdetect -y 1
```

Log out and back in after changing group membership. `i2cdetect` should show `40` for a board
using the PCA9685 default address. Use the detected address if soldered address jumpers changed
it.

## Build the workspace

Place this repository at `~/ros2/whisky_bot_ws`, then install package dependencies and build:

```bash
cd ~/ros2/whisky_bot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  whisky_bot_hardware \
  whisky_bot_description \
  whisky_bot_moveit_config \
  whisky_bot_bringup
source install/setup.bash
```

Use the normal Raspberry Pi user account to build and run the workspace. Do not use `sudo` for
`colcon`; switching to a root shell makes `~` resolve to `/root` instead of `/home/luis` and
creates root-owned build artifacts. If a prior root build created such files, restore ownership
before building as `luis`:

```bash
sudo chown -R luis:luis ~/ros2/whisky_bot_ws/build ~/ros2/whisky_bot_ws/install ~/ros2/whisky_bot_ws/log
```

Before building, verify that the ROS Jazzy environment and the hardware interface package are
available:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix hardware_interface
```

The command must print a path such as `/opt/ros/jazzy`. If it reports that the package is missing,
install the required control packages, source Jazzy again, and rebuild:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-hardware-interface \
  ros-jazzy-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-ros2-controllers \
  libconsole-bridge-dev

source /opt/ros/jazzy/setup.bash
cd ~/ros2/whisky_bot_ws
colcon build --packages-select \
  whisky_bot_hardware \
  whisky_bot_description \
  whisky_bot_bringup
```

If CMake still cannot find `console_bridge` even though `libconsole-bridge-dev` is installed,
clear its stale CMake cache and pass the multiarch CMake configuration directory explicitly:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luis/ros2/whisky_bot_ws
CONSOLE_BRIDGE_DIR="/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/console_bridge/cmake"
test -f "$CONSOLE_BRIDGE_DIR/console_bridge-config.cmake"

colcon build --cmake-clean-cache \
  --packages-select whisky_bot_hardware whisky_bot_description whisky_bot_moveit_config whisky_bot_bringup \
  --cmake-args -Dconsole_bridge_DIR="$CONSOLE_BRIDGE_DIR"
```

## Source the ROS environments

Before building or launching, source Jazzy and then the workspace overlay, in that order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/whisky_bot_ws/install/setup.bash
```

When working as `root`, do not use `~` for the workspace because it expands to `/root`. Use:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luis/ros2/whisky_bot_ws/install/setup.bash
```

Verify that the overlay is available before launching:

```bash
ros2 pkg prefix whisky_bot_bringup
```

It must print the workspace install path, for example
`/home/luis/ros2/whisky_bot_ws/install/whisky_bot_bringup`. If it only searches
`/opt/ros/jazzy`, the workspace was not sourced or its build did not finish successfully.

## Calibrate before enabling hardware

The real PCA9685 mode intentionally fails closed until calibration is completed. Edit
`src/whisky_bot_description/urdf/whisky_bot.ros2_control.xacro` and set these values for every
joint:

- `pulse_min_us` and `pulse_max_us`: measured safe PWM endpoints for that servo and linkage.
- `home_position_rad`: a mechanically safe activation pose.
- `invert`: `true` when increasing the ROS joint angle must decrease the servo pulse.

Keep `home_position_rad` inside that joint's configured position range. Do not use generic
servo pulse limits without testing them for the specific servo and mechanical arm range.

The hardware plugin checks that pulse limits are positive and ordered, the home pose is in range,
and PCA9685 channels are unique. It will not activate if any of these checks fail.

## Run

First confirm that simulation remains functional. It is the default:

```bash
ros2 launch whisky_bot_bringup whisky_bot.launch.xml
```

After I2C setup and calibration, start the physical PCA9685 hardware:

```bash
ros2 launch whisky_bot_bringup whisky_bot.launch.xml \
  use_mock_hardware:=false \
  i2c_bus:=1 \
  i2c_address:=0x40
```

The `i2c_bus` and `i2c_address` arguments are configurable. `i2c_address` accepts decimal or
hexadecimal notation, such as `0x40`.

On activation, the driver immediately commands each calibrated `home_position_rad`. Test one
servo channel at a time with the arm unloaded or mechanically restrained before sending
trajectories from MoveIt or `arm_controller`.

## Send a joint trajectory

After bringup, `arm_controller` exposes the standard
`/arm_controller/follow_joint_trajectory` action. Send a goal directly from the ROS 2 CLI to
move to a joint-space pose:

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint2, joint4, joint6, joint8, joint9],
    points: [{
      positions: [1.5708, -1.5708, 0.3854, -0.3854, -0.3854],
      time_from_start: {sec: 5}
    }]
  }}"
```

All positions are in radians and must remain within the URDF joint ranges. The example takes five
seconds to move to the provisional neutral pose. Use a long duration and test with the arm
unloaded or restrained first.

To send a two-pose trajectory, add a second point with a later `time_from_start`:

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint2, joint4, joint6, joint8, joint9],
    points: [
      {positions: [1.5708, -1.5708, 0.3854, -0.3854, -0.3854], time_from_start: {sec: 5}},
      {positions: [1.2, -1.3, 0.2, -0.2, -0.2], time_from_start: {sec: 10}}
    ]
  }}"
```

This sends a trajectory directly to `arm_controller`; it does not ask MoveIt to plan a
collision-free path. Use MoveIt through RViz or a MoveIt Python/C++ node when planning is
required. The PCA9685 is open loop, so successful action feedback means commands were accepted,
not that the physical arm reached the requested position.

### Console troubleshooting

The launch defaults to mock hardware. A successful trajectory action against mock hardware only
updates simulated state and will not move physical servos. Start the real driver explicitly:

```bash
ros2 launch whisky_bot_bringup whisky_bot.launch.xml \
  use_mock_hardware:=false \
  i2c_bus:=1 \
  i2c_address:=0x40
```

In a second terminal that sources the same ROS environments, check the hardware and controller
states:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luis/ros2/whisky_bot_ws/install/setup.bash

ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /joint_states --once
```

Expect `WhiskyBotHardwareInterface` to be active and loaded from
`whisky_bot_hardware/Pca9685SystemInterface`; `arm_controller` and
`joint_state_broadcaster` must be active. If the loaded plugin is
`mock_components/GenericSystem`, restart the launch with `use_mock_hardware:=false`.

The example trajectory above equals the suggested provisional home pose. Since the driver
commands that home pose during activation, it may not create a visible movement. With servo horns
detached or the arm mechanically safe, test a small different target:

```bash
ros2 action send_goal --feedback \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint2, joint4, joint6, joint8, joint9],
    points: [{
      positions: [1.4, -1.5708, 0.3854, -0.3854, -0.3854],
      time_from_start: {sec: 5}
    }]
  }}"
```

Before launching physical hardware, verify the I2C device and board from the Pi host (or from the
container with `/dev/i2c-1` passed through):

```bash
ls -l /dev/i2c-1
i2cdetect -y 1
```

The scan should show the configured address, normally `40`. Driver configuration, I2C, and write
errors are printed in the terminal running `ros2 launch`.

### Controller manager waits for `robot_description`

`ros2_control_node` may briefly print a warning that it is waiting for
`robot_description` while `robot_state_publisher` starts. It must disappear after startup. If it
persists, use a second sourced terminal to inspect the topic:

```bash
ros2 node list
ros2 node info /robot_state_publisher
ros2 topic info /robot_description -v
```

`/robot_state_publisher` must exist, and `/robot_description` must have a publisher and a
controller-manager subscription. Install `ros-jazzy-robot-state-publisher` if the publisher node
is missing. This launch explicitly remaps the controller manager's private
`~/robot_description` subscription to `/robot_description` and delays
`robot_state_publisher` by one second so it publishes only after the controller-manager
subscription exists.

After changing the launch file, rebuild and re-source the overlay before launching:

```bash
cd /home/luis/ros2/whisky_bot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select whisky_bot_bringup
source install/setup.bash
```

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `/dev/i2c-1` is missing | Enable I2C, reboot, and verify the Pi boot configuration. |
| `i2cdetect` does not show the board | Check SDA/SCL wiring, common ground, board power, and the selected bus. |
| `Could not open /dev/i2c-1` | Confirm the device exists and the current user belongs to the `i2c` group. |
| `Could not select PCA9685 address` | Pass the detected address with `i2c_address:=0xNN`. |
| Hardware fails while parsing calibration | Replace the zero pulse placeholders and verify all home positions and limits. |
| Servos move in the wrong direction | Set the affected joint's `invert` parameter to `true`, then retest safely. |
| `Package 'whisky_bot_bringup' not found` | Source `/opt/ros/jazzy/setup.bash`, then source `install/setup.bash`. Rebuild if `ros2 pkg prefix whisky_bot_bringup` fails. |
| `file not found: ... 'xacro'` | Install `ros-jazzy-xacro`, then source `/opt/ros/jazzy/setup.bash` again. |
| `No module named 'moveit_configs_utils'` | Install `ros-jazzy-moveit` and `ros-jazzy-moveit-configs-utils`, then source `/opt/ros/jazzy/setup.bash` again. |
| `console_bridge` cannot be found during build | Ensure `libconsole-bridge-dev` is installed, then use the explicit `console_bridge_DIR` fallback in the build section. |
