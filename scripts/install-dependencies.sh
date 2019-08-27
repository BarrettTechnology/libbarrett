#!/usr/bin/env bash
# This script installs the following dependencies:
#
# GCC 7.4
# Libconfig-Patched
# Boost 1.65.1
# Eigen 3.2.10

# Import the Linux Version Number Variables
. /etc/lsb-release

if [ "$DISTRIB_RELEASE" == "18.04" ]; then

	sudo apt-get install -y build-essential python-dev python-argparse git cmake
	sudo apt-get install -y libgsl0-dev libncurses5-dev pkg-config libboost-all-dev

	#Download and Install patched Libconfig 1.4.5
	cd ~/Downloads
	wget http://web.barrett.com/support/WAM_Installer/libconfig-1.4.5-PATCHED.tar.gz
	tar -xf libconfig-1.4.5-PATCHED.tar.gz
	cd libconfig-1.4.5
	sudo ./configure && sudo make && sudo make install

	#Download and install Eigen 3.2.10
	cd ~/Downloads
	wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
	tar --bzip2 -xf 3.2.10.tar.bz2
	cd eigen-eigen-b9cd8366d4e8/
	mkdir -p build && cd build
	cmake ../ && make && sudo make install

else
	echo "This script must run under Ubuntu 18.04"
fi

