libbarrett -- README
Barrett Technology
Updated Feb 25, 2016

Libbarrett is a real-time controls library written in C++ that runs Barrett
Technology's products, including the WAM Arm, Proficio and the BH8-280/282 BarrettHand. For
support, please email:
    support@barrett.com

To build and install libbarrett

Source based install

Pre-requisites:

$ sudo apt-get install python-dev python-argparse
$ sudo apt-get install libeigen2-dev libboost-all-dev libgsl0-dev
$ sudo apt-get install libxenomai-dev libxenomai1
$ wget http://web.barrett.com/support/WAM_Installer/libconfig-1.4.5-PATCHED.tar.gz
$ tar -xf libconfig-1.4.5-PATCHED.tar.gz
$ cd libconfig-1.4.5
$ ./configure && make && sudo make install
$ cd ../
$ rm -rf libconfig-1.4.5 libconfig-1.4.5-PATCHED.tar.gz

Download and install libbarrett:

$ cd ~/
$ git clone https://github.com/BarrettTechnology/libbarrett.git
$ cd libbarrett
$ cmake .
$ make
$ sudo make install

Configuration Files for the robots:

Upon installation of libbarrett, the configuration files of the robot are installed to the /etc/barrett directory. However, to give an additional flexibility of each user maintaining their own configurations for the same robot, by default, the configuration files are read from ~/.barrett directory if it exists. If not, then libbarrett reads the necessary configuration files from /etc/barrett/ directory. It is upto the user to maintain and populate the ~/.barrett directory

For Proficio support only:

In addition to the above, there are two different configuration files for the Proficio to account for its variant.. So, the right set of configuration files have to be copied either to the /etc/barrett/ or ~/.barrett folder depending on the configuration of the robot in use. In order to ease this process, there is a bash aliases that copies the corresponding configuration files to the /etc/barrett directory. As before, copying it to the ~/.barrett directory has to be done manually by the user.

To set it up for the first time after installing libbarrett,

Copy the bash_aliases to the existing aliases

$ cat .bash_aliases >> ~/libbarrett/.bash_aliases

or if no bash aliases exist then create a new one

$ mv libbarrett/.bash_aliases .

Rerun the bash script

$ . ~/.bashrc

- Before running the Proficio in a particular configuration or immediately after switching the configuration of the proficio, type either of the following based on the configuration of the robot

$ leftConfig
or
$ rightConfig

The above bash aliases copies the particular configuration of the proficio from ~/libbarrett/proficio_sandbox/configurations into /etc/barrett directory.

- Hit E-STOP and shift+idle.

- If the outer elbow of the proficio is swapped, do gravity calibration before running the examples.

P.S Some of the examples above may not work with any robot if the libbarrett is not installed from this source.

Headers and shared libraries will be installed to their typical locations for
your system. Configuration files will be installed to the /etc/barrett/
directory. A copy of the examples/ directory will be placed in your home
folder.

To get started, look through libbarrett's example code. For additional
documentation, see:
    http://support.barrett.com/            - Barrett product support site
    http://barrett.com/support.htm         - File download area
    http://web.barrett.com/libbarrett/     - API documentation for libbarrett

It is possible to use CMake to generate several output formats, including
Eclipse CDT4 project files. For details, see:
    http://www.paraview.org/Wiki/Eclipse_CDT4_Generator
To generate Eclipse project files, run:
    cmake . -G"Eclipse CDT4 - Unix Makefiles"
Then import the generated project into your Eclipse workspace using:
    File -> Import -> General -> Existing Projects into Workspace

In order to use libbarrett as a hard real time library, you must have the
Xenomai (http://www.xenomai.org/) real time co-kernel and its RTSocketCAN
driver installed on your system. If a hard real time guarantee is not important
for your application, you may use the SocketCAN driver from the standard Linux
kernel. To use the Linux SocketCAN driver, add "-DNON_REALTIME=true" to your
cmake command.

Additional Makefile targets include:
    make install_config      # Update or install configuration files only
    make package             # Package the library as a tar-ball

This version of libbarrett is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as published by the
Free Software Foundation.


Contact us at:
support@barrett.com
http://www.barrett.com/
+1-617-252-9000

Barrett Technology 
73 Chapel Street
Newton, MA 02458
USA
