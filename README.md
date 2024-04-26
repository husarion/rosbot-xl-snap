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
# snap connections rosbot-xl
Interface                     Plug                           Slot                                     Notes
content[ros-humble-ros-base]  rosbot-xl:ros-humble-ros-base  ros-humble-ros-base:ros-humble-ros-base  -
network                       rosbot-xl:network              :network                                 -
network-bind                  rosbot-xl:network-bind         :network-bind                            -
raw-usb                       rosbot-xl:raw-usb              :raw-usb                                 manual
shared-memory                 rosbot-xl:shm-plug             rosbot-xl:shm-slot                       manual
```

The interface `ros-humble-ros-base` must be connected.

If it isn't, you can issue the following command,

```bash
snap connect rosbot-xl:ros-humble-ros-base ros-humble-ros-base:ros-humble-ros-base
```

### Parameters

#### ROS Driver

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

To start the ROSbot XL daemon:

```bash
sudo snap start --enable rosbot-xl
```

#### ROS envs

To set `ROS_LOCALHOST_ONLY` and `ROS_DOMAIN_ID` envs use the following parameters:
- ros-localhost-only
- ros-domain-id

#### DDS Transport

The snap works with the following DDS configurations: `udp` (default), `shm` (shared memory) and `builtin`.

```bash
sudo snap set rosbot-xl transport=shm
```

Shared memory communication is possible only between the instances of ROS 2 nodes launched by the same user. Because the `rosbot-xl` snap daemon runs as `root` to allow the host <> snap communication you need to:

1. Create `/usr/local/etc/shm-only.xml` file:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>shm_transport</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="SHMParticipant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>shm_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

2. Run:

```bash
sudo su
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/etc/shm-only.xml
```

and now you should be able to list ROSbot XL ROS 2 topics with:

```bash
ros2 topic list
```

#### DDS Transport (custom)

If you want to use a custom FastDDS transport place it under the `/var/snap/rosbot-xl/common/` path (eg. `/var/snap/rosbot-xl/common/my-fastdds.xml`), and set the parameter:

```bash
sudo snap set rosbot-xl fastdds-default-profiles-file=my-fastdds.xml
```

### STM32 Firmware flashing

```bash
sudo snap connect rosbot-xl:raw-usb
sudo snap set rosbot-xl serial-port=/dev/ttyUSB0 # /dev/ttyUSBDB default - the serial port to which the digital board is connected
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

## Use

The snap ships a daemon which is automatically started once the snap is installed and configured.
Therefore, there is nothing else to do than to start using your rosbot-xl.

> **Note**
> This snap is part of an integrated snaps deployment.
> 
> Other recommended snaps to be installed are,
> 
> - [sllidar-ros2](https://snapcraft.io/sllidar-ros2)
> - [rosbot-xl-teleop](https://snapcraft.io/rosbot-xl-teleop)
> - [rosbot-xl-nav](https://snapcraft.io/rosbot-xl-nav)

