#!/usr/bin/env bash
# This script installs libbarrett dependencies

# Import the Linux Version Number Variables
. /etc/lsb-release

if [ "$DISTRIB_RELEASE" = "20.04" ] || [ "$DISTRIB_RELEASE" = "24.04" ]; then
	# Install CAN scripts
	sudo cp reset_can.sh /bin
	sudo cp 60-can.rules /etc/udev/rules.d
	sudo udevadm control --reload-rules && udevadm trigger

	# Install libbarrett dependencies
	sudo apt update
	sudo apt install -y linux-lowlatency
	sudo apt install -y git cmake clang net-tools can-utils
	sudo apt install -y libgsl-dev libeigen3-dev libncurses-dev pkg-config
	sudo apt install -y libboost-system-dev libboost-thread-dev libboost-python-dev

	# Pin the new kernel (to avoid recompiling custom modules)
	sudo apt-mark hold linux-lowlatency
	
	# Set the lowlatency kernel as the default in grub (survive apt upgrades)
	menu=`grep menuentry.*gnulinux-advanced /boot/grub/grub.cfg -m 1 |cut -d\' -f4`
	item=`grep menuentry.*lowlatency /boot/grub/grub.cfg -m 1 |cut -d\' -f4`
	sudo sed -i "s/GRUB_DEFAULT=.*/GRUB_DEFAULT='$menu>$item'/g" /etc/default/grub
	# Workaround for the 'hwmatch' bug in grub2-efi
	grep -qF GRUB_GFXPAYLOAD_LINUX /etc/default/grub || echo 'GRUB_GFXPAYLOAD_LINUX=keep' | sudo tee -a /etc/default/grub > /dev/null
	sudo update-grub

	# libbarrett now works with stock libconfig (no Barrett patch needed)
	sudo apt install -y libconfig-dev libconfig++-dev libgtest-dev

	if [ "$DISTRIB_RELEASE" = "20.04" ]; then
		# 20.04's libgtest-dev ships sources only; build and install the static libs
		cmake -S /usr/src/googletest -B /tmp/gtest-build -DCMAKE_BUILD_TYPE=Release
		make -C /tmp/gtest-build -j$(nproc)
		sudo cp /tmp/gtest-build/lib/*.a /usr/lib
	fi
else
	echo "This script must run under Ubuntu 20.04 or 24.04"
fi

