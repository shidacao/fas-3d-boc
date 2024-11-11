# Introcudtion to FAS-3D-BOC simulation

The FAS-3D-BOC simulation is a part of the paper manuscirpt "Three-dimensional bearing-only circumnavigation based on fully actuated system approach". In the simulation name, "FAS", "3D", "BOC" are short for "**F**ully **A**ctuated **S**ystem", "**3**-**D**imensional", "**B**earing-**O**nly **C**ircumnavigation", respectively.

This an introduction to how to reproduce this simulation.

## About PX4

.....................

For more information, please refer to the PX4 official Documentation website https://docs.px4.io/main/en/index.html .

## Install PX4

The authors use Ubuntu 22.04. For PX4 compilation, platforms Ubuntu 18.04/20.04/22.04, Windows 10/11 (via WSL 2) and MacOS will support.

We give a very brief introduction to the PX4 installation process in Ubuntu 22.04. For other platforms, the details are different but overall they are similar.

1. Download PX4 source code from GitHub. By default, the master branch is cloned to your computer.

	```bash
	git clone https://github.com/PX4/PX4-Autopilot.git --recursive
	```

2. Install the necessary toolchain. The following command automatically installs all the dependencies that PX4 needs for compilation.

	```bash
	bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
	```

3. Build the PX4 Software. There are several choices for simulator such as jMavSim and Gazebo, etc. The authors use jMAVSim simulator. The build command is as follows.

	```bash
	make px4_sitl jmavsim
	```

	Here, "sitl" is short for "**s**oftware **i**n **t**he **l**oop". With the "px4_sitl" option, the make target will compile the POSIX host build. With the "jmavsim" option, the lightweight simulator jMAVSim designed for PX4 is used.

This is a very rough introduction and only the skelton of the installing process is shown. For detailed information about the installation of PX4, the readers can recommanded to see https://docs.px4.io/main/en/dev_setup/dev_env.html and its child pages.

## Add FAS-3D-BOC to PX4 as an application

