#!/usr/bin/bash

# Define the source and destination directories
site_path="$SNAP/usr/local/www/foxglove/"
site_tmp_path="$SNAP_DATA/www/foxglove/"

# Ensure the destination directory exists
rm -rf $site_tmp_path
mkdir -p "$site_tmp_path"

# Copy all files and directories recursively from site_path to site_tmp_path
cp -a "$site_path"/* "$site_tmp_path"

index_html=$(cat $site_tmp_path/index.html)

replace_pattern='/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/'
replace_value=$(cat $SNAP/usr/share/$SNAP_NAME/config/foxglove-layout.json)
echo "${index_html/"$replace_pattern"/$replace_value}" > $site_tmp_path/index.html

# disable cache
sed -i "s|<div id=\"root\"></div>|<script>\nlocalStorage.clear();sessionStorage.clear();\n</script>\n&|" $site_tmp_path/index.html

# Define a function to log and echo messages
caddy run --config $SNAP/usr/share/$SNAP_NAME/config/Caddyfile --adapter caddyfile

