YDLIDAR ROS PACKAGE V1.4.0
=====================================================================

ROS node and test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build ydlidar_node and ydlidar_client
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ roscd ydlidar/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

How to run YDLIDAR ros package
=====================================================================
There're two ways to run YDLIDAR ros package

1. Run YDLIDAR node and view in the rviz
------------------------------------------------------------
roslaunch ydlidar lidar_view.launch

You should see YDLIDAR's scan result in the rviz.

2. Run YDLIDAR node and view using test application
------------------------------------------------------------
roslaunch ydlidar lidar.launch

rosrun ydlidar ydlidar_client

You should see YDLIDAR's scan result in the console


Parameters
=====================================================================
port (string, default: /dev/ydlidar)

    serial port name used in your system.


frame_id (string, default: laser_frame)

    frame ID for the device.

reversion (bool, default: false)

    indicated whether the LIDAR IS reversion.

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution.

angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data.

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data.

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data.

range_max (double, default: 8.0)

    Max valid range (m) for LIDAR data.

ignore_array (string, default: "")

    Set the current angle range value to zero.




Upgrade Log
=====================================================================

2019-04-09 version:1.4.0

   1.only support TS2 lidar
