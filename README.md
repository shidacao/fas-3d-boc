# The fas3dboc paper

This is the repository of the following fas3dboc paper.  

> "Three-dimensional bearing-only circumnavigation based on fully actuated system approach, submitted to *ISA Transactions*".

In the paper's name, "fas", "3d", "boc" are short for "**f**ully **a**ctuated **s**ystem", "**3**-**d**imensional", "**b**earing-**o**nly **c**ircumnavigation", respectively.

## About the paper

This part will be added after paper acception.

## PX4 SITL simulation of fas3dboc

The PX4 SITL (Software In The Loop) simulation of this paper is contained here. 
This an introduction to how to reproduce this simulation.

### About PX4

PX4 is a well-known autopilot for many kinds of vehicles and is widely used in universities and enterprises.
For more information, please refer to the PX4 official documentation website https://docs.px4.io/main/en/index.html .

### Install PX4

For PX4 compilation, it is supported on platforms such as Ubuntu 18.04/20.04/22.04, Windows 10/11 (via WSL2), and macOS.
The authors use Ubuntu 22.04. 
We give a very brief introduction to the PX4 installation process in Ubuntu 22.04. 
For other platforms, the details are different but overall they are similar.

1. Download PX4 source code from GitHub. By default, the master branch is cloned to your computer. Until the authors finish their paper, the master branch of PX4 is v1.15.

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

	With the "px4_sitl" option, the make target will compile the POSIX host build. With the "jmavsim" option, the lightweight simulator jMAVSim designed for PX4 is used.

This is a very rough introduction and only the skelton of the installing process is shown. For detailed information about the installation of PX4, the readers can recommanded to see https://docs.px4.io/main/en/dev_setup/dev_env.html and its child pages.

### Add fas-3d-boc module to PX4 as an application







