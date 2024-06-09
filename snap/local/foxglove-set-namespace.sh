#!/bin/bash

sudo cp /var/snap/rosbot-xl/common/foxglove-sensors.json ./test1.json
sudo cp /var/snap/rosbot-xl/common/foxglove-sensors.json ./test.json

# yq -i  '(.configById[] | select(has("paths"))).paths[].value = "abc" | 
#         (.configById[] | select(has("topicName"))).topicName = "abc"' test.json

yq -i '(.configById[] | select(has("paths"))).paths[].value |= "/abc" + .' test.json
yq -i '(.configById[] | select(has("topicName"))).topicName |= "/abc" + .' test.json
yq -i '(.configById[] | select(has("topics"))).topics |= with_entries(.key |= "/abc" + .)' test.json
# yq -i '(.configById[] | with_entries(select(.key | test(".*Topic$"))).value |= "/abc" + .)' test.json
# yq -i '(.configById[] | select(has("publish")) | .publish | with_entries(select(.key | test(".*Topic$")) | .value) |= "/abc" + .)' test.json
# yq -i '(.configById[] | select(has("publish")) | .publish | with_entries(select(.key | test(".*Topic$")) | .value |= "/abc" + .))' test.json
