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

	if [ "$DISTRIB_RELEASE" = "20.04" ]; then
		# Download and Install patched Libconfig 1.4.5 (supporting C & C++ simultaneously)
		cd ~/Downloads && wget http://web.barrett.com/support/WAM_Installer/libconfig-1.4.5-PATCHED.tar.gz
		tar -xf libconfig-1.4.5-PATCHED.tar.gz
		cd libconfig-1.4.5 && ./configure && make -j$(nproc)
		sudo make install
		sudo ldconfig
	else
		# libbarrett now works with stock libconfig (no Barrett patch needed)
		sudo apt install -y libconfig-dev libconfig++-dev libgtest-dev
	fi
else
	echo "This script must run under Ubuntu 20.04 or 24.04"
fi

