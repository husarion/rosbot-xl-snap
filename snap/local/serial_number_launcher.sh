#!/bin/bash -e

output="$(
ros2 service call /get_cpu_id std_srvs/srv/Trigger
)"

# Extract the cpu_id from the output
cpu_id=$(echo "$output" | grep -oP '"cpu_id": "\K[^"]+')

# Print the cpu_id
print-serial-number.py $cpu_id