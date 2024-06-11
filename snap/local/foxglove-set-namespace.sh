#!/bin/bash -e

LAYOUT="$(snapctl get webui.layout)"
NAMESPACE="$(snapctl get driver.namespace)"

cp ${SNAP_COMMON}/foxglove-${LAYOUT}.json ${SNAP_COMMON}/foxglove-layout.json

if [ -n "$NAMESPACE" ]; then
    # Update path fields
    yq -i -o=json '(.configById[] | select(has("path")) | .path) |= "/'$NAMESPACE'" + .' ${SNAP_COMMON}/foxglove-layout.json

    # Update paths[].value fields
    yq -i -o=json '(.configById[] | select(has("paths")) | .paths[].value) |= "/'$NAMESPACE'" + .' ${SNAP_COMMON}/foxglove-layout.json

    # Update topicName fields
    yq -i -o=json '(.configById[] | select(has("topicName")) | .topicName) |= "/'$NAMESPACE'" + .' ${SNAP_COMMON}/foxglove-layout.json

    # Update topic fields
    yq -i -o=json '(.configById[] | select(has("topic")) | .topic) |= "/'$NAMESPACE'" + .' ${SNAP_COMMON}/foxglove-layout.json

    # Update keys within topics
    yq -i -o=json '(.configById[] | select(has("topics")) | .topics) |= with_entries(.key |= "/'$NAMESPACE'" + .)' ${SNAP_COMMON}/foxglove-layout.json

    # Update values of keys matching .*Topic$
    yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("poseTopic")) | .poseTopic) |= "/'$NAMESPACE'" + .'  ${SNAP_COMMON}/foxglove-layout.json
    yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("pointTopic")) | .pointTopic) |= "/'$NAMESPACE'" + .'  ${SNAP_COMMON}/foxglove-layout.json
    yq -i -o=json '(.configById[] | select(has("publish")) | .publish | select(has("poseEstimateTopic")) | .poseEstimateTopic) |= "/'$NAMESPACE'" + .'  ${SNAP_COMMON}/foxglove-layout.json

    # Update imageTopic
    yq -i -o=json '(.configById[] | select(has("imageMode")) | .imageMode | select(has("imageTopic")) | .imageTopic) |= "/'$NAMESPACE'" + .'  ${SNAP_COMMON}/foxglove-layout.json
fi
