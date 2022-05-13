# Autonomous Quadrotor Control Library (AQC or AQCL)

Description/Purpose

The Autonomous Quadrotor Control Library (henceforth known as AQC), was created with the intention of making software development for autonomous quadrotors *much* easier. This library currently only targets the [PX4-Autopilot](https://px4.io/) software, but could be extended to additionally support [Ardupilot](https://ardupilot.org/). Additionally, despite its name, this library supports any multicopter that is supported by PX4.

At its core, this library exists to serve two main purposes:
1. Provide a framework within which to easily (and quickly!) develop deployable software that targets real, physical hardware.
2. Provide an interface that is agnostic to the output so that all of your code can remain exactly the same (with the exception of a single launch file) when switching between simulation and physical hardware.


#### README Sections:
1. [Demo Videos](#demo-videos)
2. [Software Description](#software)
	1. [Dependencies](#software-dependencies)
	2. [SITL Simulation](#configuring-and-running-sitl-simulation)
	3. [Using an Existing Input Client](#how-to-use-an-existing-input-client)
	4. [Writing a New Input Client](#how-to-write-a-new-input-client)
3. [Hardware Description](#hardware)
	1. [Requirements](#hardware-requirements)
	2. [Running on Hardware](#configuring-and-running-on-hardware)
4. [Future Work](#future-work)
5. [Overview of Each Package](#overview-of-each-package)


## Demo Videos

1. [Brief Hardware Tour](https://www.youtube.com/watch?v=1UGx2_VTam8&t=11s)
2. [Demo]()


## Software

### Software Dependencies

#### 1. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) running on [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04/)

#### 2. [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

This is a ground control software that enables communication between a "ground control station" (GCS) and the flight controller unit (FCU). It also provides an easy interface to update firmware and/or configuration parameters of the FCU, and an extremely helpful GUI that displays a lot of information about the vehicle during flight (both in real life and in simulation). Note that the latest stable version of QGC only officially supports Ubuntu 20.0.4 and later due to video streaming issues, but I have had experienced no errors thus far. If you do encounter problems, older stable versions can be downloaded [here](https://github.com/mavlink/qgroundcontrol/releases/).

To install: 
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
```

Log out and log back in for user permissions to change  
Download [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage)

To run:
```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

#### 3. [PX4-Autopilot](https://docs.px4.io/master/en/dev_setup/building_px4.html)

This is the autopilot software that will be running on the FCU with real hardware, and that will be running "in-the-loop" during simulation. Alternatively, you could use [Ardupilot](#https://ardupilot.org/), but that will not be compatible with this repository.

To install:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

#### 4. [mavros](http://wiki.ros.org/mavros)

In short, this provides the necessary bridge for communication between ROS and the Pixhawk. In slightly more detail, the PX4-Autopilot software running on the Pixhawk communicates using the Mavlink protocol commonly used on drones (although I believe it is now capable of communicating with ROS2 directly). Mavros simply provides the bridge from any ROS communications to Mavlink communications.

To install:
```bash
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
```

#### 5. [GeographicLibDatasets](https://geographiclib.sourceforge.io/)

These are geographic datasets that are utilized by the ground control software.

To install:
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### Configuring and Running SITL Simulation

```bash
$ cd
```

### How to Use an Existing Input Client


### How to Write a New Input Client



## Hardware

This section will focus exclusively on the hardware setup that I used for testing this library; however, there are infinitely many other hardware combinations (e.g. choice of airframe, companion computer, FCU, radios, etc.) that would be supported by this library.

### Hardware Requirements

requirements:
1. Companion Computer -- [NVIDIA Jetson Nano 2GB](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/education-projects/)
2. Multicopter -- I used a [Holybro x500 v2 Quadrotor](http://www.holybro.com/product/x500-v2-kit/)
3. FCU -- I used a [Holybro Pixhawk5x](http://www.holybro.com/product/pixhawk-5x/)
4. GPS -- [Holyrbo M9N](http://www.holybro.com/product/holybro-m9n-gps/)
5. Telemetry radios -- I used [mRobotics 915 Mhz](https://store.mrobotics.io/mRo-SiK-Telemetry-Radio-V2-915Mhz-p/m10013-rk.htm)
6. RC receiver/transmitter -- [FlySky FS-i6](https://www.flysky-cn.com/i6-gaishu)
7. UART Connection -- [USB-to-TTL](https://www.amazon.com/dp/B07D6LLX19?psc=1&ref=ppx_yo2ov_dt_b_product_details)
8. Wifi adapter -- 802.11ac wifi adapter
9. Power -- lipo plus other peripherals (and jetson battery)


### Configuring and Running on Hardware 



## Future Work


