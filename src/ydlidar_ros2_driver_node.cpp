/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port", str_optvalue);
  node->get_parameter("port", str_optvalue);
  ///lidar port
  laser.setSerialPort(str_optvalue);

  ///ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array", str_optvalue);
  node->get_parameter("ignore_array", str_optvalue);
  std::vector<float> ignore_array;
  // Parse ignore array string if needed
  laser.setIgnoreArray(ignore_array);

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  node->declare_parameter("baudrate", optval);
  node->get_parameter("baudrate", optval);
  laser.setSerialBaudrate(optval);
  /// tof lidar
  optval = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", optval);
  node->get_parameter("lidar_type", optval);
  laser.setLidarType(optval);
  /// device type
  // NOTE: setDeviceType() removed in latest SDK - serial is default
  optval = 0; // YDLIDAR_TYPE_SERIAL equivalent
  node->declare_parameter("device_type", optval);
  node->get_parameter("device_type", optval);
  // laser.setDeviceType(optval); // Method doesn't exist in new SDK
  /// sample rate
  optval = 9;
  node->declare_parameter("sample_rate", optval);
  node->get_parameter("sample_rate", optval);
  laser.setSampleRate(optval);
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count", optval);
  node->get_parameter("abnormal_check_count", optval);
  laser.setAbnormalCheckCount(optval);

  /// Intenstiy bit count
  // NOTE: setIntensityBit() removed in latest SDK
  optval = 0;
  node->declare_parameter("intensity_bit", optval);
  node->get_parameter("intensity_bit", optval);
  // laser.setIntensityBit(optval); // Method doesn't exist in new SDK
     
  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  node->declare_parameter("fixed_resolution", b_optvalue);
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setFixedResolution(b_optvalue);
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion", b_optvalue);
  node->get_parameter("reversion", b_optvalue);
  laser.setReversion(b_optvalue);
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted", b_optvalue);
  node->get_parameter("inverted", b_optvalue);
  laser.setInverted(b_optvalue);
  b_optvalue = true;
  node->declare_parameter("auto_reconnect", b_optvalue);
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setAutoReconnect(b_optvalue);
  /// one-way communication
  b_optvalue = false;
  node->declare_parameter("isSingleChannel", b_optvalue);
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setSingleChannel(b_optvalue);
  /// intensity
  // NOTE: setIntensity() removed in latest SDK - G4 doesn't support intensity anyway
  b_optvalue = false;
  node->declare_parameter("intensity", b_optvalue);
  node->get_parameter("intensity", b_optvalue);
  // laser.setIntensity(b_optvalue); // Method doesn't exist in new SDK
  /// Motor DTR
  // NOTE: setSupportMotorDtrCtrl() removed in latest SDK
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr", b_optvalue);
  node->get_parameter("support_motor_dtr", b_optvalue);
  // laser.setSupportMotorDtrCtrl(b_optvalue); // Method doesn't exist in new SDK
  //是否启用调试
  // NOTE: setEnableDebug() removed in latest SDK
  b_optvalue = false;
  node->declare_parameter("debug", b_optvalue);
  node->get_parameter("debug", b_optvalue);
  // laser.setEnableDebug(b_optvalue); // Method doesn't exist in new SDK

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max", f_optvalue);
  node->get_parameter("angle_max", f_optvalue);
  laser.setMaxAngle(f_optvalue);
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min", f_optvalue);
  node->get_parameter("angle_min", f_optvalue);
  laser.setMinAngle(f_optvalue);
  /// unit: m
  f_optvalue = 64.f;
  node->declare_parameter("range_max", f_optvalue);
  node->get_parameter("range_max", f_optvalue);
  laser.setMaxRange(f_optvalue);
  f_optvalue = 0.1f;
  node->declare_parameter("range_min", f_optvalue);
  node->get_parameter("range_min", f_optvalue);
  laser.setMinRange(f_optvalue);
  /// unit: Hz
  f_optvalue = 10.f;
  node->declare_parameter("frequency", f_optvalue);
  node->get_parameter("frequency", f_optvalue);
  laser.setScanFrequency(f_optvalue);

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);


  bool ret = laser.initialize();
  if (ret)
  {
    //设置GS工作模式（非GS雷达请无视该代码）
    // Note: setWorkMode() API removed in new SDK, functionality may be integrated into initialize()
    int i_v = 0;
    node->declare_parameter("m1_mode", i_v);
    node->get_parameter("m1_mode", i_v);
    i_v = 0;
    node->declare_parameter("m2_mode", i_v);
    node->get_parameter("m2_mode", i_v);
    i_v = 1;
    node->declare_parameter("m3_mode", i_v);
    node->get_parameter("m3_mode", i_v);
    //启动扫描
    ret = laser.turnOn();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize lidar\n");
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", rclcpp::SensorDataQoS());
  
  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//
    bool hardwareError = false;

    if (laser.doProcessSimple(scan, hardwareError)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      pc_msg->header = scan_msg->header;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      pc_msg->channels.resize(2);
      int idx_intensity = 0;
      pc_msg->channels[idx_intensity].name = "intensities";
      int idx_timestamp = 1;
      pc_msg->channels[idx_timestamp].name = "stamps";

      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
	  if (scan.points[i].range >= scan.config.min_range) {
            scan_msg->ranges[index] = scan.points[i].range;
            scan_msg->intensities[index] = scan.points[i].intensity;
	  }
        }

	if (scan.points[i].range >= scan.config.min_range &&
             scan.points[i].range <= scan.config.max_range) {
          geometry_msgs::msg::Point32 point;
          point.x = scan.points[i].range * cos(scan.points[i].angle);
          point.y = scan.points[i].range * sin(scan.points[i].angle);
          point.z = 0.0;
          pc_msg->points.push_back(point);
          pc_msg->channels[idx_intensity].values.push_back(scan.points[i].intensity);
          pc_msg->channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
        }

      }

      laser_pub->publish(*scan_msg);
      pc_pub->publish(*pc_msg);

    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
