# rosbot-xl-snap

This snap packages the [`rosbot_xl_ros`](https://github.com/husarion/rosbot_xl_ros) package.
It thus conveniently offers all the ROS 2 stack necessary to bring up the [ROSbot-XL](https://husarion.com/manuals/rosbot-xl/) robot,
including IMU driver, robot state publisher, joint state publisher, controllers and more.

[![Get it from the Snap Store](https://snapcraft.io/static/images/badges/en/snap-store-black.svg)](https://snapcraft.io/rosbot-xl)

## Installation

Install the snap as follows,

```bash
snap install rosbot-xl
```

## Setup

Upon installation, depending on your operating system,
you may have to manually connect the snap interface.
You can verify that with the following command,

```bash
$ snap connections rosbot-xl
Interface            Plug                    Slot                            Notes
content[ros-humble]  rosbot-xl:ros-humble    ros-humble-ros-base:ros-humble  manual
network              rosbot-xl:network       :network                        -
network-bind         rosbot-xl:network-bind  :network-bind                   -
```

The interface `ros-humble` must be connected.

If it isn't, you can issue the following command,

```bash
snap connect rosbot-xl:ros-humble ros-humble-ros-base
```

### Parameters

Depending on your robot hardware,
the snap can be configured through the following parameters:

- mecanum
- include-camera-mount
- camera-model
- lidar-model

which can be set as follows, e.g.,

```bash
snap set rosbot-xl mecanum=True
```

## Use

The snap ships a daemon which is automatically started once the snap is installed and configured.
Therefore, there is nothing else to do than to start using your rosbot-xl.

Note that this snap is part of an integrated snaps deployment.
Other recommended snaps to be installed are,

- [micro-xrce-dds-agent](LINK)
- [sllidar-ros2](https://snapcraft.io/sllidar-ros2)
- [rosbot-xl-teleop](https://snapcraft.io/rosbot-xl-teleop)
- [rosbot-xl-nav](https://snapcraft.io/rosbot-xl-nav)
