# rosbot-xl-snap

This snap packages the [`rosbot_xl_ros`](https://github.com/husarion/rosbot_xl_ros) package. It thus conveniently offers all the ROS 2 stack necessary to bring up the [ROSbot-XL](https://husarion.com/manuals/rosbot-xl/) robot, including IMU driver, robot state publisher, joint state publisher, controllers and more.

[![Get it from the Snap Store](https://snapcraft.io/static/images/badges/en/snap-store-black.svg)](https://snapcraft.io/rosbot-xl)

## Installation

Install the snap as follows,

```bash
snap install rosbot-xl
```

## Parameters

```bash
$ sudo snap get rosbot-xl
Key                 Value
driver              {...}
ros-domain-id       123
ros-localhost-only  0
serial-port         auto
transport           udp
webui               {...}
```

### ROS Driver

Parameters for [rosbot_xl_ros](https://github.com/husarion/rosbot_xl_ros) ROS 2 driver available from the snap level are:

```bash
Key                          Value
driver.camera-model          (unset)
driver.include-camera-mount  True
driver.mecanum               True
driver.lidar-model           (unset)
driver.namesapce             (unset)
```

which can be set as follows, e.g.,

```bash
snap set rosbot-xl driver.mecanum=True
```

### Web UI

```bash
Key           Value
webui.layout  sensors
webui.port    8080
```

The snap has a built-in Web UI that can be launched with:

```bash
sudo rosbot-xl.start-web-ui
```

By default the UI is available under `http://<ROSBOT_IP>:8080/ui`.

Built-in layouts are available here:
- `/var/snap/rosbot-xl/common/foxglove-default.json`
- `/var/snap/rosbot-xl/common/foxglove-sensors.json`

If you want to save your own custom layout, save it in the same directory, eg. in `/var/snap/rosbot-xl/common/foxglove-myui.json` file.

To select the layout run:

```bash
sudo snap set rosbot-xl webui.layout=myui
```
## Working With Parameters

```bash
# set params in rosbot-xl snap
sudo snap set rosbot-xl transport=udp ros-domain-id=123 driver.namespace=abc

# mirror the setup for other snaps running ROS 2:
sudo snap set husarion-depthai $(cat /var/snap/rosbot-xl/common/ros_snap_args)
sudo snap set husarion-rplidar $(cat /var/snap/rosbot-xl/common/ros_snap_args)

# to setup the current shell with the same configs:
source /var/snap/rosbot-xl/common/ros.env

# ... or install configs with
/var/snap/rosbot-xl/common/manage_ros_env.sh
# and after changing rosbot-xl params open a new terminal, or run:
source ~/.bashrc

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap __ns:=/${ROS_NAMESPACE}
```

## Custom DDS Transport 

If you want to use a custom FastDDS transport place it under the `/var/snap/rosbot-xl/common/` path (eg. `/var/snap/rosbot-xl/common/my-fastdds.xml`), and set the parameter:

```bash
sudo snap set rosbot-xl transport=my-fastdds
```

## STM32 Firmware flashing

```bash
sudo snap set rosbot-xl serial-port=/dev/ttyUSB0 # 'auto' is default option - the serial port to which the digital board is connected
sudo rosbot-xl.flash
```

## Logs

To display logs from the ROS 2 node:

```bash
sudo snap logs rosbot-xl
```

To display logs from a snap logger

```bash
journalctl -t rosbot-xl
```

> **Note**
> This snap is part of an integrated snaps deployment.
> 
> Other recommended snaps to be installed are,
> - [husarion-rplidar](https://snapcraft.io/husarion-rplidar)
> - [husarion-depthai](https://snapcraft.io/husarion-depthai)
> - [rosbot-xl-teleop](https://snapcraft.io/rosbot-xl-teleop)
> - [rosbot-xl-nav](https://snapcraft.io/rosbot-xl-nav)

<!-- [sllidar-ros2](https://snapcraft.io/sllidar-ros2) -->
