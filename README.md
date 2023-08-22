# Advanced Navigation ROS2 Driver - IR Version

## Introduction

This is an example using the Advanced Navigation Spatial SDK to create a ROS2 driver that reads and decodes the Advanced Navigation Packet Protocol (ANPP) (in this case packet #20 and packet #28) and publishes the information as ROS topics / messages. 

This example also includes the encoding of ANPP packet #55 and pushes the information to the Spatial INS.

It is designed to work with all Advanced Navigation INS devices using ANPP.

The code has been written to be easy to understand and for ease of extensibility with other ANPP packets.

This example has been developed and tested using **Ubuntu Linux v20.04 LTS** and **ROS2 Humble**. Installation instructions for ROS2 can be found here: https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

If you require any assistance using this code, please email support@advancednavigation.com


## Build Instruction

- Packages should be created in the src directory, not the root of the workspace. Navigate to `workspace-folder-name/src`, and get the Advanced Navigation ROS2 Driver   
  ```
  git clone https://github.com/independentrobotics/advanced-navigation-orientus-driver.git
  ```
- You likely already have the `rclpp` and `std_msgs` packages installed as part of your ROS2 system. Either way, it’s good practice to run rosdep in the root of your workspace (`workspace-folder-name`) to check for missing dependencies before building:
  ```
  rosdep install -i --from-path src --rosdistro humble -y
  ```
- In the root of your workspace, `workspace-folder-name`, source and build the package:
  - Source the ROS2 Environment to the current folder:
    ```
    source /opt/ros/humble/setup.bash
    ```
  - Build your new package:
    ```
    colcon build --packages-select advanced-navigation-orientus
    ```

## Device Configuration

To use this example code, your Advanced Navigation device should be configured to output ANPP packets #20 and #28.

If you are not sure how to configure your Advanced Navigation Device please refer to the Reference Manual on the relevant product page (https://www.advancednavigation.com/products/all). 



## Run Instructions

Open a new terminal or new tab, navigate to `workspace-folder-name`, and source the setup files:
```
source /opt/ros/humble/setup.bash
. install/setup.bash
```

- Run the Driver in the following manners:
  1. Baud Rate and Comm Port as arguments:
     ```
     usage: ros2 run package_name executable_name [baud_rate] [comm_port]
        package_name     Name of the ROS package
        executable_name  Name of the executable
        baud_rate        The Baud rate configured on the device. Default 115200
        comm_port        The COM port of the connected device. Default /dev/ttyUSB0
     ```
     ***e.g. ros2 run ros2-driver adnav_driver 115200 /dev/ttyUSB0***
  2. Baud Rate, Comm Port and NTRIP as arguments:
     ```
     ros2 run package_name executable_name -B [baud_rate] -D [comm_port] -s [server_url] -m [mountpoint]  -u [username] -p [password]
       package_name     Name of the ROS package
       executable_name  Name of the executable
       -B baud_rate     Baud rate configured on the device. Default 115200
       -D comm_port     COM port of the connected device. Default /dev/ttyUSB
       -s server_url    URL of the NTRIP server
       -m mountpoint    Name of the mountpoint
       -u username      Username of your NTRIP account
       -p password      Password of your NTRIP account 
     ```
     ***e.g. ros2 run ros2-driver adnav_driver -B 115200 -D /dev/ttyUSB0 -s alldayrtk.com -m MOUNTPOINT_20  -u yourUsername -p yourPassword***


## Published Topics
Use RQT Monitor to view published topics. Here you will find details on how to use RQT: https://index.ros.org/doc/ros2/Tutorials/RQt-Overview-Usage/
- Open a new terminal or new tab and source ROS2 Environment to the current folder:
  ```
  source /opt/ros/foxy/setup.bash
  ```
- Run RQT Monitor by entering the following:
  ```
  rqt
  ```
- To view the published messages in RQT Monitor, click **Plugins** and click **Topic Monitor**
