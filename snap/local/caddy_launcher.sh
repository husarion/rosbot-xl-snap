#!/usr/bin/bash

# Define a function to log and echo messages
log_and_echo() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "caddy_launcher: $message"
    # Echo the message to standard error
    echo >&2 "$message"
}

# Define the source and destination directories
site_path="$SNAP/usr/local/www/"
site_tmp_path="$SNAP_DATA/www/"
layout="$(snapctl get webui.layout)"
log_and_echo "Using ${SNAP_COMMON}/$layout"

# Ensure the destination directory exists
rm -rf $site_tmp_path
mkdir -p "$site_tmp_path"

# Copy foxglove folder to temp path
cp -R $site_path/foxglove $site_tmp_path

# apply the layout
index_html_path=$site_tmp_path/foxglove/index.html
index_html_content=$(cat $index_html_path)
replace_pattern='/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/'
replace_value=$(cat ${SNAP_COMMON}/$layout)
echo "${index_html_content/"$replace_pattern"/$replace_value}" > $index_html_path

# disable cache
sed -i "s|<div id=\"root\"></div>|<script>\nlocalStorage.clear();sessionStorage.clear();\n</script>\n&|" $index_html_path

# Define a function to log and echo messages
caddy run --config $SNAP/usr/share/$SNAP_NAME/config/Caddyfile --adapter caddyfile

