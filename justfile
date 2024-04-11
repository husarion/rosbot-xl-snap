dev-build:
    #!/bin/bash
    export SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1
    sudo /bin/bash -c "echo "net.ipv4.conf.all.forwarding=1" > /etc/sysctl.d/99-forwarding.conf"
    sudo systemctl stop docker.service
    sudo systemctl stop docker.socket
    sudo systemctl restart systemd-sysctl

    sudo iptables -I DOCKER-USER -i lxdbr0 -j ACCEPT
    sudo iptables -I DOCKER-USER -o lxdbr0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
    # sudo lxd init --minimal
    # sudo usermod -aG lxd $USER
    # newgrp lxd
    # snapcraft --debug
    snapcraft

dev-clean:
    #!/bin/bash
    export SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1
    snapcraft clean

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then 
            echo "Please run as root to install dependencies"
            exit 1
        fi

        sudo apt update && sudo apt install -y rsync
    fi

dev-install:
    #!/bin/bash
    if ls rosbot-xl_*.snap 1> /dev/null 2>&1; then
        sudo snap remove --purge rosbot-xl
        sudo rm -rf ./squashfs-root
        unsquashfs ./rosbot-xl_*.snap
        sudo snap try squashfs-root
    else
        echo "No snap found in current directory. Build it at first (run: just build)"
        exit 1
    fi

install:
    #!/bin/bash
    if ls rosbot-xl_*.snap 1> /dev/null 2>&1; then
        sudo snap remove --purge rosbot-xl
        sudo snap install ./rosbot-xl_*.snap --dangerous
    else
        echo "No snap found in current directory. Build it at first (run: just build)"
        exit 1
    fi

dev-launch:
    #!/bin/bash
    export SERIAL_PORT=/dev/ttyUSBDB
    # export SERIAL_PORT_SLOT=$(snap interface serial-port | yq .slots[0] | sed 's/^\([^ ]*\) .*/\1/')
    export SERIAL_PORT_SLOT="snapd:ft230xbasicuart"
    
    sudo snap set rosbot-xl serial-port=$SERIAL_PORT 
    sudo snap connect rosbot-xl:serial-port $SERIAL_PORT_SLOT
    sudo snap connect rosbot:ros-humble ros-humble-ros-base
    sudo snap start --enable rosbot-xl # with --enable it will start after reboot
    # sudo snap logs rosbot-xl
    # sudo snap stop rosbot-xl
    # sudo snap restart rosbot-xl

stop:
    #!/bin/bash
    sudo snap disconnect rosbot-xl:ros-humble

info:
    #!/bin/bash
    sudo snap get rosbot-xl
    sudo snap connections rosbot-xl

logs:
    #!/bin/bash
    sudo snap logs rosbot-xl -n 20

# copy repo content to remote host with 'rsync' and watch for changes
sync hostname password="husarion": _install-rsync
    #!/bin/bash
    sshpass -p "husarion" rsync -vRr ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ ; do
        sshpass -p "{{password}}" rsync -vRr ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

    # sshpass -p "husarion" rsync -vRr --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    # while inotifywait -r -e modify,create,delete,move ./ ; do
    #     sshpass -p "{{password}}" rsync -vRr --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    # done


install-snap:
    #!/bin/bash
    sudo apt install snapd

    sudo systemctl unmask snapd.service
    sudo systemctl unmask snapd.socket
    sudo systemctl unmask snapd.seeded.service

    sudo systemctl enable snapd.service
    sudo systemctl enable snapd.socket
    sudo systemctl enable snapd.seeded.service

    sudo systemctl start snapd.service
    sudo systemctl start snapd.socket
    sudo systemctl start snapd.seeded.service

install-snapcraft:
    #!/bin/bash
    # fixing issues with networking (https://documentation.ubuntu.com/lxd/en/latest/howto/network_bridge_firewalld/?_ga=2.178752743.25601779.1705486119-1059592906.1705486119#prevent-connectivity-issues-with-lxd-and-docker)
    sudo bash -c 'echo "net.ipv4.conf.all.forwarding=1" > /etc/sysctl.d/99-forwarding.conf'
    sudo systemctl stop docker.service
    sudo systemctl stop docker.socket
    sudo systemctl restart systemd-sysctl
    # now you may need to reboot (after reboot docker will not work until you run `sudo systemctl start docker.service`)

    sudo snap install lxd
    sudo lxd init --minimal
    sudo snap install snapcraft --classic

    # Enable the hotplug feature and restart the `snapd` daemon (for serial interface):
    sudo snap set core experimental.hotplug=true
    sudo systemctl restart snapd

    sudo usermod -aG lxd $USER
    newgrp lxd


enable-docker:
    #!/bin/bash
    sudo systemctl start docker.service
    sudo systemctl start docker.socket
    sudo systemctl restart systemd-sysctl

# run teleop_twist_keybaord (host)
teleop:
    #!/bin/bash
    # export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/shm-only.xml
    ros2 run teleop_twist_keyboard teleop_twist_keyboard # --ros-args -r __ns:=/robot
