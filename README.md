YDLIDAR ROS PACKAGE V2.0.0
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
------------------------------------------------------------
port (string, default: /dev/ydlidar)

    serial port name used in your system.

frame_id (string, default: laser_frame)

    frame ID for the device.

reversion (bool, default: false)

    indicated whether the LIDAR IS reversion.
    
 sun_noise (bool, default: true)

    indicated whether to turn off solar noise
    
glass_noise (bool, default: true)

    indicated whether to turn off glass noise.

calibration_filename (string, default: LidarAngleCalibration.ini)

        Lidar zero angle calibration file.

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution.

angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data.

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data.

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data.

range_max (double, default: 16.0)

    Max valid range (m) for LIDAR data.

ignore_array (string, default: "")

    Set the current angle range value to zero.

frequency (double, default: 7)

    the LIDAR scanning frequency.




Upgrade Log
------------------------------------------------------------

2019-01-03 version:2.0.0

   1.Remove other lidar model interfaces functions.

   2.fix turnOn function.
   
   3.Lidar supports zero offset angle adjustment.

2018-11-24 version:1.3.8

   1.Reduce abnormal situation recovery time.
   
   2.fix timestamp from zero.

2018-10-29 version:1.3.7

   1.Update SDK verison to 1.3.6

   2.add calibration zero angle.

2018-06-19 version:1.3.3

   1.Update SDK verison to 1.3.5

   2.add debug MODEL

   3.add reversion scan data

2018-04-16 version:1.3.1

   1.Update SDK verison to 1.3.1

   2.Increase sampling frequency,scan frequency setting.

   3.Unified coordinate system.

   4.Repair X4,S4 LIDAR cannot be opened.

   5.Increased G4 G4C F4Pro LIDAR power-off protection.

   6.Increased S4B LIDAR low optical power setting.

   7.Fix the wait time for closing ros node.

   8.Compensate for each laser point timestamp.

   9.Unified profile, automatic correction lidar model.
   
      
   Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)
