#!/bin/bash -e

# Define a function to log and echo messages
source $SNAP/usr/bin/utils.sh

# Define the source and destination directories
site_path="$SNAP/usr/local/www/"
site_tmp_path="$SNAP_DATA/www/"
layout="$(snapctl get webui.layout)"
# layout="layout"
export UI_PORT="$(snapctl get webui.port)"
log_and_echo "Using ${SNAP_COMMON}/foxglove-$layout.json"

# Ensure the destination directory exists
rm -rf $site_tmp_path
mkdir -p "$site_tmp_path"

# Copy foxglove folder to temp path
cp -R $site_path/foxglove $site_tmp_path

# apply the layout
index_html_path=$site_tmp_path/foxglove/index.html
index_html_content=$(cat $index_html_path)
replace_pattern='/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/'
replace_value=$(cat ${SNAP_COMMON}/foxglove-$layout.json)
echo "${index_html_content/"$replace_pattern"/$replace_value}" > $index_html_path

# disable cache
sed -i "s|<div id=\"root\"></div>|<script>\nlocalStorage.clear();sessionStorage.clear();\n</script>\n&|" $index_html_path

# Define a function to log and echo messages
# Retrieve the ros.namespace value
namespace=$(snapctl get ros.namespace)

# Check if the namespace is set and not empty
if [ -n "$namespace" ]; then
  # Set the environment variable
  export NAMESPACE="/$namespace"
fi
caddy run --config $SNAP/usr/share/$SNAP_NAME/config/Caddyfile --adapter caddyfile

