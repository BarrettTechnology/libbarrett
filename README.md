# Libbarrett 2.0.0
> Libbarrett is a real-time controls library written in C++ that runs Barrett
Technology products (WAM and BarrettHand)

` This version of Libbarrett compiles at non-real-time kernel (standard Ubuntu 18.04 kernel) and should be used when hard real-time guarantee is not important for your application.`


### Download package
```
cd ~/
git clone https://git.barrett.com/software/libbarrett.git
cd libbarrett
```

### Install depedencies
Navigate to the scripts folder:
```
cd scripts
```
Install the required dependencies:
```
./install-dependencies.sh
```

Setup your hardware for the communication with the CAN bus and reboot your system:
```
./setup-hardware.sh
sudo reboot
```

### Compile Libbarrett in non-realtime fashion
```
cmake -DNON_REALTIME=true .
make
sudo make install
```

### Additional Makefile targets
Update or install configuration files only
```
make install_config
```
Package the library as a tar-ball 
```
make package
```

### Configuration Files for the WAM
Upon installation of libbarrett, the configuration files of the robot are installed to the `/etc/barrett` directory. However, to give an additional flexibility of each user maintaining their own configurations for the same robot, by default, the configuration files are read from `~/.barrett` directory if it exists. If not, then libbarrett reads the necessary configuration files from `/etc/barrett/` directory. It is up to the user to maintain and populate the `~/.barrett` directory.

### Eclipse IDE setup
It is possible to use CMake to generate several output formats, including Eclipse CDT4 project files. For details, see:
    http://www.paraview.org/Wiki/Eclipse_CDT4_Generator
To generate Eclipse project files, run:
```
cmake . -G"Eclipse CDT4 - Unix Makefiles"
```
Then import the generated project into your Eclipse workspace using:
File -> Import -> General -> Existing Projects into Workspace
