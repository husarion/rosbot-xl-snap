#!/bin/bash

# place in /usr/local/sbin

# Define a function to log messages
source $SNAP/usr/bin/utils.sh

SERVER_IP=192.168.77.2
SERVER_PORT=3000

fifo_path="/tmp/response"
rm -f "$fifo_path"
mkfifo "$fifo_path"

# Define a path for the shutdown flag file
SHUTDOWN_FLAG="/tmp/shutdown_flag"
rm -f "$SHUTDOWN_FLAG"  # Ensure it does not exist initially

handleRequest() {
    while read -r line; do
        echo "$line"
        trline=$(echo "$line" | tr -d '[\r\n]')

        [ -z "$trline" ] && break

        HEADLINE_REGEX='(.*?)\s(.*?)\sHTTP.*?'

        if [[ "$trline" =~ $HEADLINE_REGEX ]]; then
            REQUEST=$(echo "$trline" | sed -E "s/$HEADLINE_REGEX/\1 \2/")
        fi
    done

    case "$REQUEST" in
    "GET /ping")
        RESPONSE="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\npong"
        ;;
    "GET /shutdown")
        RESPONSE="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\nshutting down"
        touch "$SHUTDOWN_FLAG"  # Create the flag file to signal shutdown
        ;;
    *)
        RESPONSE="HTTP/1.1 404 NotFound\r\nContent-Type: text/html\r\n\r\nNot Found"
        ;;
    esac

    echo -e "$RESPONSE" >"$fifo_path"
}

# check if we can start db-server

# Send a GET request to the server's /ping endpoint
status=$(curl --silent --max-time 1 "http://$SERVER_IP:$SERVER_PORT/ping")
if [ $? -eq 0 ]; then
    log "Received response: $status"
fi

# Check the response
if [[ "$status" == "pong" ]]; then
  log "Legacy server running on port 3000. To turn it off run:"
  log "sudo systemctl stop db-server.service && sudo systemctl disable db-server.service"
  log "After that start the service:"
  log "sudo snap start --enable ${SNAP_NAME}.db-server"

  snapctl stop --disable ${SNAP_NAME}.db-server 2>&1 || true
fi

log "Listening on $SERVER_IP:$SERVER_PORT..."

while true; do
    # Handle requests
    cat "$fifo_path" | nc -lN -s $SERVER_IP -p $SERVER_PORT | handleRequest

    if [ -f "$SHUTDOWN_FLAG" ]; then
        log "shutting down..."

        dbus-send --system --print-reply --dest=org.freedesktop.login1 \
        /org/freedesktop/login1 org.freedesktop.login1.Manager.PowerOff boolean:true
        
        break
    fi

    sleep 1
done
