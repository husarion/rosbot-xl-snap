#!/bin/bash
# place in /usr/local/sbin

# Define a function to log messages
log() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "db_server_launcher: $message"
}


rm -f response
mkfifo response

ENABLE_SHUTDOWN="false"

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
        ENABLE_SHUTDOWN="true"
        ;;
    *)
        RESPONSE="HTTP/1.1 404 NotFound\r\nContent-Type: text/html\r\n\r\nNot Found"
        ;;
    esac

    echo -e "$RESPONSE" >response
}

# check if we can start db-server

# Send a GET request to the server's /ping endpoint
status=$(curl --silent --max-time 1 "http://192.168.77.2:3000/ping")
if [ $? -eq 0 ]; then
    log "Received response: $status"
else
    log "Failed to receive response, handling error..."
    # Implement error handling logic here
fi

# Check the response
if [[ "$status" == "pong" ]]; then
  log "[!] Legacy server running on port 3000. To turn it off run:"
  log "sudo systemctl stop db-server.service && sudo systemctl disable db-server.service"
  log "After that start the service:"
  log "sudo snap start --enable ${SNAP_NAME}.db-server"

  snapctl stop --disable ${SNAP_NAME}.db-server 2>&1 || true
fi

log 'Listening on 3000...'

while true; do
    # Handle requests
    nc -l -p 3000 -s 192.168.77.2 < response | handleRequest

    sleep 1

    # Check if shutdown is enabled
    if [[ "$ENABLE_SHUTDOWN" == "true" ]]; then
        log "shutting down..."
        
        dbus-send --system --print-reply --dest=org.freedesktop.login1 \
        /org/freedesktop/login1 org.freedesktop.login1.Manager.PowerOff boolean:true
        break
    fi
done
