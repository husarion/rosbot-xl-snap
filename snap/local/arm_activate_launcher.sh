#!/bin/bash -e

STATE=$1

ros2 service call /controller_manager/set_hardware_component_state \
    controller_manager_msgs/srv/SetHardwareComponentState \
    "{name: 'manipulator', target_state: {id: 0, label: '${STATE}'}}"