# Advanced Navigation ROS2 Driver

## Introduction

This is an example using the Advanced Navigation SDK to create a ROS2 driver that reads and decodes the Advanced Navigation Packet Protocol (ANPP) Packets and publishes the information as ROS topics / messages. 

This example is currently able to decode the following packets into ROS messages;
- ANPP 0: Acknowledge Packet
- ANPP 3: Device Information Packet
- ANPP 20: System State Packet
- ANPP 28: Raw Sensors Packet
- ANPP 33: ECEF Position Packet  

Further packets can be requested and logged, however without extension these will not be added to ROS Messages or the terminal.

This driver is designed to work with all Advanced Navigation INS devices using ANPP, and is able to interface through both serial and IP communication methods.

The code has been written to be easy to understand and for ease of extensibility with other ANPP packets. If you wish to extend the driver please see the [Extending the Driver](#extending-the-driver) Section. 

This example has been developed and tested using **Ubuntu Linux v22.04 LTS** and **ROS2 Humble Hawksbill**. Installation instructions for ROS2 can be found [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html "Installation -- ROS 2 Documentation"): 

If you require any assistance using this driver, please email [support@advancednavigation.com](mailto:support@advancednavigation.com)



## ROS2 Getting Started Guide

The following guides are useful in getting started with ROS2 if you are not familiar:

- [ROS 2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html "Tutorials -- ROS 2 Documentation")
- [Setting up a ROS2 Workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/ "Creating a Workspace")
- [Basic Tutorial on importing an Example ROS2 code and compiling and running](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/ "Writing a Simple CPP Publisher and Subscriber")


## Build Instruction

- Packages should be created in the src directory, not the root of the workspace. Navigate to `workspace-folder-name/src`, and clone this repository into it.
  ```
  $ git clone TODO ADD URL
  ```
- You likely already have the `rclcpp` and `_msgs` packages installed as part of your ROS2 system. Either way, it’s good practice to run rosdep in the root of your workspace (`workspace-folder-name`) to check for missing dependencies before building:
  ```
  $ rosdep install -i --from-path src --rosdistro humble -y
  ```
- In the root of your workspace, `workspace-folder-name`, source and build the package:
  - Source the ROS2 Environment to the current folder:
    ```
    $ source /opt/ros/humble/setup.bash
    ```
  - Build your package:
    ```
    $ colcon build
    ```
    This will build three adnav specific packages;
    
    - adnav_driver - The main driver for your adnav_device
    - adnav_interfaces - Messages and Service definitions for the driver
    - adnav_launch - Launch files and configuration for the driver

  - You should now source the built package in the terminal: 
    ```
    $ source install/setup.bash
    ```



## Device and Driver Configuration

To use the driver in your intended configuration it is recommended to use a launch file from the adnav_launch package. From your `workspace-folder-name` navigate to the adnav_launch package using the following:

```
$ cd src/adnav-ros2/adnav_launch/
```
We can examine the structure of the package by using the following:
```
$ ls -R
.:
CMakeLists.txt  config  launch  package.xml

./config:
adnav_serial.yaml      adnav_tcp_server.yaml
adnav_tcp_client.yaml  adnav_udp_client.yaml

./launch:
adnav_serial.launch.py      adnav_tcp_server.launch.py
adnav_tcp_client.launch.py  adnav_udp_client.launch.py
```
From this we can see two folders `config` and `launch`. The launch folder contains python launch files that will start the driver using the named communication method and data from the `.yaml` files in the config folder.

You can create your own launch files for this driver by copying one, and editing it using the following:

```
$ cp config/adnav_serial.yaml config/custom.yaml
$ cp launch/adnav_serial.launch.py launch/custom.launch.py && nano launch/custom.launch.py
```
When the GNU nano editor opens you will notice a config section that looks like the following:
```
config = os.path.join(
        get_package_share_directory('adnav_launch'),
        'config',
        'adnav_serial.yaml'
    )
```
Changing `adnav_serial.yaml` to `custom.yaml` will link your new launch file to your configuration. 

From this we can edit the configuration file to change behaviour of the device and driver:

```
$ nano config/custom.yaml
```

These parameters can be changed as you wish. For example we can create a 50Hz serial communicator by changing the `publish_us` field to 20000.

Saving this file and navigating to `workspace-folder-name` we can rebuild the launch package. 
```
$ colcon build --packages-select adnav_launch
```

After sourcing the `setup.bash` once more you can launch the driver with your new configuration. 
```
$ source install/setup.bash
$ ros2 launch adnav_launch custom.launch.py
```

We can then verify the rate of output by running the following:
```
$ ros2 topic hz /imu
```

## Run Instructions

Open a new terminal or new tab, navigate to `workspace-folder-name`, and source the setup files:
```
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
```

- Launch the Driver in the following manner:
     ```
     $ ros2 launch adnav_launch adnav_serial.launch.py
     ```
  This will use the preset serial configuration to open communication with the device and start publishing messages. 

You should then see the following in your terminal

```
...
[adnav_driver-1] [INFO] [1673829196.793632251] [adnav_node]: Your Advanced Navigation ROS driver is currently running
[adnav_driver-1] Press Ctrl-C to interrupt
...
```


## Published Topics and Service Servers
You can view the topics published to by the driver using the `ros2 topic` command.

- Open a new terminal and navigate to your `workspace-folder-name` and source your ROS install and workspace:
  ```
  $ source /opt/ros/humble/setup.bash
  $ source install/setup.bash
  ```
- Launch the driver
  ```
  $ ros2 launch adnav_launch adnav_serial.launch.py
  ```
- Then in a new terminal run the following to see the topics published to by the driver
  ```
  $ ros2 node info /adnav_node
  /adnav_node
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
    Publishers:
      /barometric_pressure: sensor_msgs/msg/FluidPressure
      /filter_status: diagnostic_msgs/msg/DiagnosticStatus
      /gnss_fix_type: std_msgs/msg/String
      /imu: sensor_msgs/msg/Imu
      /magnetic_field: sensor_msgs/msg/MagneticField
      /nav_sat_fix: sensor_msgs/msg/NavSatFix
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /pose: geometry_msgs/msg/Pose
      /rosout: rcl_interfaces/msg/Log
      /system_status: diagnostic_msgs/msg/DiagnosticStatus
      /tempurature: sensor_msgs/msg/Temperature
      /twist: geometry_msgs/msg/Twist
    Service Servers:
      /adnav_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /adnav_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /adnav_node/get_parameters: rcl_interfaces/srv/GetParameters
      /adnav_node/list_parameters: rcl_interfaces/srv/ListParameters
      /adnav_node/ntrip: adnav_interfaces/srv/Ntrip
      /adnav_node/packet_periods: adnav_interfaces/srv/PacketPeriods
      /adnav_node/packet_timer_period: adnav_interfaces/srv/PacketTimerPeriod
      /adnav_node/request_packet: adnav_interfaces/srv/RequestPacket
      /adnav_node/restart_publishing: std_srvs/srv/Empty
      /adnav_node/restart_reading: std_srvs/srv/Empty
      /adnav_node/set_parameters: rcl_interfaces/srv/SetParameters
      /adnav_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:

    Action Clients:

  ```

## Extending the Driver
To extend the functionality of the driver to include more ANPP packets you will need to create a new decode function and add it to the driver. We will use ANPP 40 the Quaternion Orientation Packet as an example. 

- First navigate to the adnav_driver folder from `workspace-folder-name`
  ```
  $ cd src/adnav-ros2/adnav_driver/
  ```
- Then open `advanced_navigation_driver.h` in your preferred editor, such as nano. 
  ```
  $ nano include/adnav_driver/advanced_navigation_driver.h
  ```

- Then add a definition for a new RosDriver function inside the AdvancedNavigationDriver class that takes a pointer to an `an_packet_t` struct.
  ```
  void quatOrientRosDecoder(an_packet_t* an_packet);
  ```

- After saving open up the `advanced_navigation_driver.cpp` file in your favourite editor.
  ```
  $ nano src/advanced_navigation_driver.cpp
  ```

- Add the function we just defined into the file. 
  ```
  void AdvancedNavigationDriver::quatOrientRosDecoder(an_packet_t* an_packet){
    // Make packet structure
    quaternion_orientation_packet_t quaternion_orientation_packet;

    // Get lock on the ROS Messages
    std::unique_lock<std::mutex> lock(messages_mutex_);

    // ECEF Position (in meters) Packet for Pose Message
    if(decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
    {
      pose_msg_.orientation.w = quaternion_orientation_packet.orientation[0];
      pose_msg_.orientation.x = quaternion_orientation_packet.orientation[1];
      pose_msg_.orientation.y = quaternion_orientation_packet.orientation[2];
      pose_msg_.orientation.z = quaternion_orientation_packet.orientation[3];

      pose_msg_.orientation.w = quaternion_orientation_packet.orientation[0];
      pose_msg_.orientation.x = quaternion_orientation_packet.orientation[1];
      pose_msg_.orientation.y = quaternion_orientation_packet.orientation[2];
      pose_msg_.orientation.z = quaternion_orientation_packet.orientation[3];
    }else{
      RCLCPP_ERROR(this->get_logger(), "Error Decoding Packet 40");
    }	

    // Notify and update Condition variable for publishers
    msg_write_done_ = true;
    msg_cv_.notify_one();
  } // lock releases once outside scope.
  ```
  Lets look through this section by section.
    - We first open the function and create a struct to hold the decoded data
    ```
    void AdvancedNavigationDriver::quatOrientRosDecoder(an_packet_t* an_packet){
      // Make packet structure
      quaternion_orientation_packet_t quaternion_orientation_packet;
    ```  
    - Then we gain a lock on the messages to prevent race conditions.
    ```
      // Get lock on the ROS Messages
      std::unique_lock<std::mutex> lock(messages_mutex_);
    ```
    - Then we decode the packet using the provided function from `ins_packets.c` and place the information into relevant ROS messages (Note: there may be more than one message that is built to take the data.)
    ```
      // ECEF Position (in meters) Packet for Pose Message
      if(decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
      {
        pose_msg_.orientation.w = quaternion_orientation_packet.orientation[0];
        pose_msg_.orientation.x = quaternion_orientation_packet.orientation[1];
        pose_msg_.orientation.y = quaternion_orientation_packet.orientation[2];
        pose_msg_.orientation.z = quaternion_orientation_packet.orientation[3];

        pose_msg_.orientation.w = quaternion_orientation_packet.orientation[0];
        pose_msg_.orientation.x = quaternion_orientation_packet.orientation[1];
        pose_msg_.orientation.y = quaternion_orientation_packet.orientation[2];
        pose_msg_.orientation.z = quaternion_orientation_packet.orientation[3];
      }else{
        RCLCPP_ERROR(this->get_logger(), "Error Decoding Packet 40");
      }	
    ```

    - Then we update the condition variable and notify the publishing threads that an update has been made. 
    ```
    msg_write_done_ = true;
    msg_cv_.notify_one();
    ```

    - Then we close the function which releases the lock back for other functions and threads to use.

- Then we must tell the decoder to use this new function when packet 40 is received. To do this we must add an extra case in the `decodePackets()` method. 
  ```
  case packet_id_quaternion_orientation: quatOrientRosDecoder(an_packet);
      break;
  ```

Once this is complete we must rebuild the driver. Navigating back to `workspace-folder-name`. 
```
$ colcon build --packages-select adnav_driver
```

Once built you can run the driver and request the newly implemented packet from the device see [Device and Driver Configuration](#device-and-driver-configuration). 


## The NTRIP Client
The NTRIP Client will send RTCM corrections data to your device using ANPP Packet 55. It will also log the raw data from the server into a .rtcm file for convienience.

To use the NTRIP Client you will need to have the following information for the client to make a connection. 
- NTRIP Host. Free examples include;
    - RTK2GO | rtk2go.com:2101
    - International GNSS Service (requires registration) | igs-ip.net:2101
- Authentication (Username and Password) if your selected server requires
- Mountpoint (Data-stream

### The Ntrip Service
- Firstly navigate to the `workspace-folder-name` and source the setup files
```
source install/setup.bash
```
- Launch the driver using your a supported communication method. 
```
ros2 launch adnav_launch adnav_<COM_METHOD>.launch.py
```
- Once the driver has started running you will see the following. 
```
[adnav_node]: Your Advanced Navigation ROS driver is currently running
Press Ctrl-C to interrupt
```
- Once the driver is running you may call the ntrip service. We can see the service provided by running the following in a new terminal.
```
$ ros2 service list
/adnav_node/describe_parameters
/adnav_node/get_parameter_types
/adnav_node/get_parameters
/adnav_node/list_parameters
/adnav_node/ntrip
/adnav_node/packet_periods
/adnav_node/packet_timer_period
/adnav_node/request_packet
/adnav_node/restart_publishing
/adnav_node/restart_reading
/adnav_node/set_parameters
/adnav_node/set_parameters_atomically
```
- From the list we can see the service `/adnav_node/ntrip`. To find more information on the service we can run the following commands.

```
$ ros2 service type /adnav_node/ntrip
adnav_interfaces/srv/Ntrip
```
- This gives us the structure for the call and response of the service. we can get more info by running the following. 
```
$ ros2 interface show adnav_interfaces/srv/Ntrip 

# This contains the data structure a service initializing the NTRIP Client of the
# Advanced Navigation ROS Drivers. If the service is called while ntrip is running
# the new configuration will override the old configuration.

bool NTRIP_ENABLE = true#
bool NTRIP_DISABLE = false#

# Boolean value describing wether to activate or deactivate the ntrip client
bool enable

# Data structure containing the required connection parameters.
# ignored if enable is false
adnav_interfaces/NtripData data
	string host
	string username
	string password
	string mountpoint

---

# Success response
bool success

# String for more in depth response on success
string reason
```
- From this we can see that there are two calling fields (enable and the adnav_interfaces/NtripData structure which holds connection information), and two responing fields (a success flag and a success reason). 
  
### Calling the NTRIP Service
The NTRIP Service is like any other service in ros2 and can be called wither by other nodes, or through the CLI. Please see the following tutorials on hwo to make a simple service client. [(C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#write-the-client-node)[(Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#write-the-client-node)  

To use the CLI you will need to provide the calling data in the service call in a new terminal using YAML syntax as shown below. Ensure your new terminal has sourced the `setup.bash` file.  

```
$ ros2 service call /adnav_node/ntrip adnav_interfaces/srv/Ntrip "{enable: true, host: 'NTRIPHostname:Port', username: 'ExampleUsername', password: 'StrongPassword123', mountpoint: 'AParticularMountpoint'}"
```

Calling the service with correct information yields the following

```
requester: making request: adnav_interfaces.srv.Ntrip_Request(enable=True, host='------------------------', username='------------------------', password='------------------------', mountpoint='------------------------')

response:
adnav_interfaces.srv.Ntrip_Response(success=True, reason='Successfuly updated NTRIP client.')
```

And in the Drivers terminal
```
NtripClient service running...
```

If the mountpoint is left blank the service will fail however back in the drivers terminal it will have printed the SOURCETABLE if the server provided it. You can then look through this table to find an appropriate mountpoint and select it. 

```
Request result: SOURCETABLE 200 OK
Server: NTRIP BKG Caster 2.0.43/2.0
Date: Thu, 12 Jan 2023 23:02:45 GMT
Connection: close
Content-Type: text/plain
Content-Length: 66402

;igs-ip.net;2101;IGS-IP;BKG;0;DEU;50.12;8.69;0.0.0.0;0;http://igs-ip.net/home
CAS;rtcm-ntrip.org;2101;NtripInfoCaster;BKG;0;DEU;50.12;8.69;0.0.0.0;0;http://rtcm-ntrip.org/home
NET;EUREF;EUREF;B;N;https://igs.bkg.bund.de/root_ftp/NTRIP/streams/streamlist_euref-ip.htm;https://igs.bkg.bund.de:443/root_ftp/EUREF/station/rnxskl/;http://register.rtcm-ntrip.org;none
NET;IGS;IGS;B;N;https://igs.bkg.bund.de/root_ftp/NTRIP/streams/streamlist_igs-ip.htm;https://igs.bkg.bund.de:443/root_ftp/IGS/station/rnxskl/;http://register.rtcm-ntrip.org;none
NET;MGEX;IGS;B;N;https://igs.bkg.bund.de/root_ftp/NTRIP/streams/streamlist_igs-ip.htm;https://igs.bkg.bund.de:443/root_ftp/MGEX/station/rnxskl/;http://register.rtcm-ntrip.org;none
NET;MISC;BKG;B;N;https://igs.bkg.bund.de/root_ftp/NTRIP/streams/streamlist_igs-ip.htm;https://igs.bkg.bund.de:443/root_ftp/MISC/station/rnxskl/;http://register.rtcm-ntrip.org;none
STR;ABMF00GLP0;Les-Abymes;RTCM 3.3;1006(30),1007(30),1008(30),1013(30),1019,1020,1033(30),1042,1045,1046,1074(1),1084(1),1094(1),1104(1),1114(1),1124(1),1230(30);2;GPS+GLO+GAL+BDS+QZS+SBAS;IGS;GLP;16.26;-61.53;0;0;SEPT POLARX5;none;B;N;10200;rgp-ip.ign.fr:2101/ABMF00GLP7(1)

---

STR;ZIM200CHE0;Zimmerwald;RTCM 3.3;1006(10),1008(10),1019,1020,1033(10),1042,1045,1046,1075(1),1085(1),1095(1),1125(1),1230(10);2;GPS+GLO+GAL+BDS;IGS;CHE;46.88;7.47;0;0;TRIMBLE NETR9;none;B;N;8800;tpp.swipos.ch:8090/ZIM2_RTCM(1)
ENDSOURCETABLE

Remote socket close!!!
```

### Disabling the NTRIP service

If you wish to disable the ntrip service after it is already running you may run the same command as used to start it but with the enable field changed to false. The other sections may be left blank as they will be ignored.
```
$ ros2 service call /adnav_node/ntrip adnav_interfaces/srv/Ntrip "{enable: false, data:{host: '', username: '', password: '', mountpoint: ''}}"
```

running this while the client is operating gives the following. 

```
requester: making request: adnav_interfaces.srv.Ntrip_Request(enable=False, host='', username='', password='', mountpoint=''))

response:
adnav_interfaces.srv.Ntrip_Response(success=True, reason='Successfuly updated NTRIP client.')
```

And in the Drivers terminal
```
NtripClient service done.
[adnav_node]: RTCM Log file for this session can be found in: Log_<HOST:PORT>_<MOUNTPOINT>_<DATE>_<TIME>.rtcm
```