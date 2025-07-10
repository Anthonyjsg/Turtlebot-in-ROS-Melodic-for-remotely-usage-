# Turtlebot in ROS Melodic (for remotely usage)

This repository provides step-by-step instructions on how to install the software needed to control the Turtlebot2 using a Raspberry Pi 2 with Ubuntu 18.04 and ROS Melodic, controlled remotely from a base computer.

## Equipment required

- Turtlebot 2
- Raspberry pi 2
- SD card > 8GB
- Powerbank

## Installing Ubuntu 18.04 on the Raspberry Pi

### Programs you will need

1. [7-Zip File Manager](https://www.7-zip.org/)
2. [SD Card Formatter](https://www.sdcard.org/downloads/formatter/)
3. [Win32DiskImager](https://win32diskimager.org/)

### Installation procedure

1. Go to [official Ubuntu Mate website](https://releases.ubuntu-mate.org/archived/18.04/armhf/?C=M&O=D) and download the Ubuntu 18.04 image for the armhf processor (.xz file). 

> NOTE: Since this is an older version, you can find it in the directory 
[**/archived/18.04/armhf/**](https://releases.ubuntu-mate.org/archived/18.04/armhf/?C=M&O=D)
> 
1. Use 7-Zip to decompress the image..
2. Insert the SD card into your computer, open the SD Card Formatter program, select the SD card, and click FORMAT.
3. Open Win32DiskImager, locate the unzipped file, select the SD card as the destination, and mount the image (click on Write).
4. Insert the card into the Raspberry, connect it to a monitor, keyboard, and mouse, and configure Ubuntu for the first time.

## Remote connection programs

### Install Remote Desktop (RDP) on Ubuntu 18.04

1. Update your system

```bash
sudo apt update && sudo apt upgrade -y
```

1. Install XRDP

```bash
sudo apt install xrdp -y
```

1. Activate and start the xrdp service

```bash
sudo systemctl enable xrdp
sudo systemctl start xrdp
```

1. Verify that it is running

```bash
sudo systemctl status xrdp
```

1. Install a desktop environment

```bash
sudo apt install xfce4 xfce4-goodies -y
```

1. Configure xrdp to use XFCE

```bash
echo "startxfce4" > ~/.xsession
```

1. Make sure xrdp uses xorg instead of vnc

```bash
sudo sed -i.bak '/^test -x \/etc\/X11\/Xsession && exec \/etc\/X11\/Xsession/s/^/#/' /etc/xrdp/startwm.sh
echo "startxfce4" | sudo tee -a /etc/xrdp/startwm.sh
```

1. Allow RDP on the firewall

```bash
sudo ufw allow 3389/tcp
```

> NOTE: If, when you reconnect and enter your credentials, the connection is rejected, it may be because the system automatically logs into the desktop environment on raspberry. To fix this, do the following:
> 

Run the command

```bash
sudo raspi-config
```

> Go to Boot options > Select B1 Desktop / CLI Choose wheter to boot into a desktop enviroment or the command line > B1 Console Text console.
> 

### Install and enable SSH on Ubuntu

1. Install the SSH server

```bash
sudo apt install openssh-server -y
```

1. Verify that the service is active with the first command, and if it is not, use the command below and check again.

```bash
sudo systemctl status ssh 
sudo systemctl start ssh
```

1. To enable it always at startup, use

```bash
sudo systemctl enable ssh
```

> NOTE: this will be useful later on for easily sending files from your computer to the Raspberry Pi using the following command:
> 

```bash
scp "Windows_file_path.py" user@IP_Raspberry:/home/ubuntu_user/
```

## Installing ROS Melodic and turtlebot packages

### Steps for installing ROS Melodic

You can find more detailed information on the [official ROS website](https://wiki.ros.org/melodic/Installation/Ubuntu), but here is a summary of the steps to follow:

1. **Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse.**

```cpp
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt update
```

1. Setup your computer to accept software from [packages.ros.org](http://packages.ros.org/)

```cpp
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

1. **Set up your keys**

```cpp
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

1. Make sure your Debian package index is up-to-date

```cpp
sudo apt update
```

1. **Desktop Install (**ROS, [rqt](https://wiki.ros.org/rqt), [rviz](https://wiki.ros.org/rviz), and robot-generic libraries)

```cpp
sudo apt install ros-melodic-desktop
```

1. It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched

```cpp
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

1. **Dependencies for building packages**

```cpp
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

1. **Initialize rosdep**

```cpp
sudo apt install python-rosdep
```

1. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Installing Turtlebot packages

This installation is based [on this tutorial](https://www.youtube.com/watch?v=rniyH8dY5t4).

1. Create the workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

1. Go to the source directory, clone the Turtlebot packages, and compile from the source directory

```bash
cd src
curl -sLf [https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_all.sh](https://www.youtube.com/redirect?event=comments&redir_token=QUFFLUhqbnJENWY2Ry0xbnZ4TlFWNkQ2U2swTE15aVg3Z3xBQ3Jtc0trSHNxb3hkOWpYaEJtOVVUcS1WV3k3NTBlNW95UVdqc3R6RnZPTEtWLUdBTkxIVWc5MlV6YjZaMGFnQmdqSDRzeHd2RFVOSVVjYnJKRlNBb21yU0ZOS0V3UHRWci1NME90dm1qN0VnSDFyNVRfd21CRQ&q=https%3A%2F%2Fraw.githubusercontent.com%2Fgaunthan%2FTurtlebot2-On-Melodic%2Fmaster%2Finstall_all.sh) | bash
cd ~/catkin_ws/
catkin_make -j1
```

1. Source the workspace

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

1. Install additional dependencies

```bash
cd ~/catkin_ws/
sudo apt-get install ros-melodic-joy*
sudo apt-get install ros-melodic-move-base*
sudo apt-get install ros-melodic-map-server*
sudo apt-get install ros-melodic-amcl*
sudo apt-get install ros-melodic-navigation*
catkin_make -j1
```

### Run ROS and check that everything is correct

Move the simulation robot and check that everything loads correctly

```bash
roslaunch turtlebot_stage turtlebot_in_stage.launch
```

### Testing the Turtlebot

1. Connect the Turtlebot 2 to the Raspberry Pi and bring it into the ROS environment.

```bash
roslaunch turtlebot_bringup minimal.launch
```

1. If you want to use keyboard to control it, just run the following command

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

### **Set the Network Time Protocol**

Clock synchronization is important for ROS. Chrony has been found to be the best ntp client over lossy wireless. In case of robot behaves strange when messages are sent from PC application(like rviz, rqt, or ros node running in PC), you need clock synchronization.

1. Install Chrony
    
    ```bash
    sudo apt-get install chrony
    ```
    
2. Manually sync NTP
    
    ```bash
    sudo ntpdate ntp.ubuntu.com
    ```
    

### Run scripts

At this point, you can create and run custom scripts so that the robot follows a specific route. In the Scripts directory of this repository, you will find some sample code.

Create a folder in the root directory called scripts

```bash
mkdir -p ~/scripts
```

To run a script, while in the directory that contains it

```bash
python mi_script.py
```