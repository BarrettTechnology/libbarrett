libbarrett -- README
Barrett Technology
2012-05-02


Libbarrett is a real-time controls library written in C++ that runs Barrett
Technology's products, including the WAM Arm and the BH8-280 BarrettHand. For
support, please email:
    support@barrett.com

To build and install libbarrett, run:
    cmake .
    make
    sudo make install
Headers and shared libraries will be installed to their typical locations for
your system. Configuration files will be installed to the /etc/barrett/
directory. A copy of the examples/ directory will be placed in your home
folder.

To get started, look through libbarrett's example code. For additional
documentation, see:
    http://support.barrett.com/            - Barrett product support site
    http://barrett.com/robot/support.htm   - File download area
    http://web.barrett.com/libbarrett/     - API documentation for libbarrett

It is possible to use CMake to generate several output formats, including
Eclipse CDT4 project files. For details, see:
    http://www.paraview.org/Wiki/Eclipse_CDT4_Generator
To generate Eclipse project files, run:
    cmake . -G"Eclipse CDT4 - Unix Makefiles"
Then import the generated project into your Eclipse workspace using:
    File -> Import -> General -> Existing Projects into Workspace

In order to use libbarrett, you must have the Xenomai (http://www.xenomai.org/)
real time co-kernel and its SocketCAN RTCAN driver installed on your system.
For additional dependencies, see:
    http://web.barrett.com/svn/libbarrett/dependencies/

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
625 Mount Auburn Street
Cambridge, MA 02138
USA
