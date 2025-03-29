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

Generally speaking, there are three methods to use the PX4 autopolit:

1. as a module of PX4;
2. with ROS: MAVROS for ROS 1 and uXRCE-DDS for ROS 2; and
3. MAVSDK.

In this simulation, we adopt the first method so our program is of the flight control level.
This means that, if it is applied to a real quadrotor, our program runs on the flight control boards such as Pixhawk and does not run on onboard computers such as Raspberry Pi.
The authors think that the above three method are all suitable for the simulation of this paper.
We just choose one way but the other two ways are also good.

Our program is a module of PX4 so the "simulation/fas_3d_boc" folder of ours should be moved to the "src/examples" path of the PX4 folder.
In the terminal in the PX4 folder's path, run

```bash
make px4_sitl jmavsim
```

Then our module is together compiled with the PX4 firmware and the jMavSim simulator is opened after compilation.
After the GPS-ready message (in green font), in the PX4's zsh shell, start our program through running

```zsh
fas_3d_boc
```

Then the quadrotor takes off, hovers in 3 meters height for 5 seconds and then performs the circumnavigation task.

The circumnavigation parameters are written in the source file "simulation/fas_3d_boc/Fas3dBoc.cpp".
The video "simulation/fas-3d-boc_sim.mp4" shows the circumnavigation result.










