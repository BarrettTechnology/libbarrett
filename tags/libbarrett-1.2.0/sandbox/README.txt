libbarrett -- Sandbox Code README
Barrett Technology
2012-07-12


Libbarrett is a real-time controls library written in C++ that runs Barrett
Technology's products, including the WAM Arm and the BH8-280 BarrettHand. For
support, please email:
    support@barrett.com

To build the sandbox programs, first install libbarrett. Then run:
    cmake .
    make

WARNING: The contents of this directory may be overwritten if you install a new
version of libbarrett. Please rename this directory before modifying the code.

To get started, look through the example code in this directory. For additional
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
