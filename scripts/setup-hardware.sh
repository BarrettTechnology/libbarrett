#!/bin/bash

# Import the Linux Version Number Variables
. /etc/lsb-release

if [ "$DISTRIB_RELEASE" == "18.04" ]; then
	sudo cp reset_can_non_xenomai.sh /bin
	sudo cp 60-can.rules /etc/udev/rules.d
	sudo udevadm control --reload-rules && udevadm trigger
else
	echo "This script must run under Ubuntu 18.04"
fi
