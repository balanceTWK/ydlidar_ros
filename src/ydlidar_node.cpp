/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "CYdLidar.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

using namespace ydlidar;

#define ROSVerision "2.0.4"


std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "ydlidar_node"); 
    printf("__   ______  _     ___ ____    _    ____     _____ _____ __  __ ___  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\   |_   _| ____|  \\/  |_ _| \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) |____| | |  _| | |\\/| || |  \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <_____| | | |___| |  | || |  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\    |_| |_____|_|  |_|___| \n");
    printf("\n");
    fflush(stdout);
  
    std::string port;
    int baudrate=921600;
    bool intensities = false;
    std::string frame_id;
    bool auto_reconnect;
    double angle_max,angle_min;
    std::string list;
    std::vector<float> ignore_array;  
    double max_range, min_range;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar"); 
    nh_private.param<int>("baudrate", baudrate, 921600);
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<double>("angle_max", angle_max , 0);
    nh_private.param<double>("angle_min", angle_min , -60);
    nh_private.param<double>("range_max", max_range , 16.0);
    nh_private.param<double>("range_min", min_range , 0.08);
    nh_private.param<std::string>("ignore_array",list,"");

    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            ROS_ERROR_STREAM("ignore array should be between -180 and 180");
        }
    }

    CYdLidar laser;
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    ROS_INFO("[YDLIDAR INFO] Now YDLIDAR ROS SDK VERSION:%s .......", ROSVerision);
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setIntensities(intensities);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setAutoReconnect(auto_reconnect);
    laser.setIgnoreArray(ignore_array);
    bool ret = laser.initialize();
    if (!ret) {
        ROS_ERROR("Error initializing YDLIDAR Comms and Status!!!");
    }
    ros::Rate rate(20);

    while (ret&&ros::ok()) {
        bool hardError;
        LaserScan scan;//
        if(laser.doProcessSimple(scan, hardError )){
            int fixed_size = scan.data.size();
            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.system_time_stamp/1000000000ul;
            start_scan_time.nsec = scan.system_time_stamp%1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min =(scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            if((scan_msg.angle_max - scan_msg.angle_min) == 2*M_PI) {
                scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / fixed_size;
            } else {
                scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (fixed_size - 1);
            }
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            scan_msg.ranges.resize(fixed_size);
            scan_msg.intensities.resize(fixed_size);

            for(int i=0 ; i < fixed_size; i++) {
                LaserPoint point = scan.data[i];
                int index = ((point.angle - scan.config.min_angle)/scan_msg.angle_increment + 0.5);
                if(index >= 0 && index <= fixed_size) {
                    scan_msg.ranges[index] = point.range;
                    scan_msg.intensities[index] = point.intensity;
                }
            }
            scan_pub.publish(scan_msg);
        }  
        rate.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    laser.disconnecting();
    return 0;
}
