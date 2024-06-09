#!/bin/bash

sudo cp /var/snap/rosbot-xl/common/foxglove-sensors.json ./test1.json
sudo cp /var/snap/rosbot-xl/common/foxglove-sensors.json ./test.json

# Update path fields
yq -i -o=json '(.configById[] | select(has("path")) | .path) |= "/xyz" + .' test.json

# Update paths[].value fields
yq -i -o=json '(.configById[] | select(has("paths")) | .paths[].value) |= "/xyz" + .' test.json

# Update topicName fields
yq -i -o=json '(.configById[] | select(has("topicName")) | .topicName) |= "/xyz" + .' test.json

# Update topic fields
yq -i -o=json '(.configById[] | select(has("topic")) | .topic) |= "/xyz" + .' test.json

# Update keys within topics
yq -i -o=json '(.configById[] | select(has("topics")) | .topics) |= with_entries(.key |= "/xyz" + .)' test.json

# Update values of keys matching .*Topic$
yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("poseTopic")) | .poseTopic) |= "/abc" + .'  test.json
yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("pointTopic")) | .pointTopic) |= "/abc" + .'  test.json
yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("poseEstimateTopic")) | .poseEstimateTopic) |= "/abc" + .'  test.json

# Update imageTopic
yq -i -o=json '(.configById[] | select(has("imageMode")) | .imageMode | select(has("imageTopic")) | .imageTopic) |= "/abc" + .'  test.json



