#!/bin/bash -e

# Function to find the ttyUSB* device for the specified USB Vendor and Product ID
find_ttyUSB() {
  # Extract vendor and product ID
  VENDOR_ID="$1"
  PRODUCT_ID="$2"

  # Get the serial-port value using snapctl
  SERIAL_PORT=$(snapctl get serial-port)

  if [ "$SERIAL_PORT" == "auto" ]; then
    for device in /sys/bus/usb/devices/*; do
      if [ -f "$device/idVendor" ]; then
        current_vendor_id=$(cat "$device/idVendor")
        if [ "$current_vendor_id" == "$VENDOR_ID" ]; then
          if [ -z "$PRODUCT_ID" ] || ([ -f "$device/idProduct" ] && [ "$(cat "$device/idProduct")" == "$PRODUCT_ID" ]); then
            # Look for ttyUSB device in the subdirectories
            for subdir in "$device/"*; do
              if [ -d "$subdir" ]; then
                for tty in $(find "$subdir" -name "ttyUSB*" -print 2>/dev/null); do
                  if [ -e "$tty" ]; then
                    ttydev=$(basename "$tty")
                    echo "/dev/$ttydev"
                    return 0
                  fi
                done
              fi
            done
          fi
        fi
      fi
    done
    echo "Error: Device with ID $VENDOR_ID:${PRODUCT_ID:-*} not found."
    return 1
  else
    echo "$SERIAL_PORT"
    return 0
  fi
}
