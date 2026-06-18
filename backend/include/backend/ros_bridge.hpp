#pragma once
#include "backend/opcua_client.hpp"
#include "backend/config.hpp"

#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <std_msgs/msg/detail/u_int8__struct.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
// #include <std_msgs/msg/int32.hpp>
//
#include <std_srvs/srv/set_bool.hpp>
#include "backend/srv/set_int16.hpp"
#include "backend/srv/set_float32.hpp"
#include "backend/srv/get_spot_weights.hpp"


using BoolPub = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;
using BoolSrv = rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr;

class RosBridge : public rclcpp::Node {
public:

explicit RosBridge(const UaConfig& cfg);

private:
  std::shared_ptr<UaClient> ua_;
  UaConfig cfg_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_speed_;
    
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sensing_slider_actual_pos_;
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cleaning_slider_actual_pos_;

// rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_welding_data_;
  // BoolPub pub_welding_data_; 
  BoolPub  pub_cobot_mode_;
  BoolPub pub_automatic_mode_;

  //sensing publishers
  BoolPub pub_sensing_robot_home_st_;
  BoolPub pub_sensing_finished_;
  BoolPub pub_touch_sensing_finished_;
  BoolPub pub_sensing_active_;
  BoolPub pub_touch_sensing_active_;
  BoolPub pub_sensing_slide_command_;
  BoolPub pub_sensing_running_;
  BoolPub pub_sensing_carbody_located_st_; 
  //cleaning publishers
  BoolPub pub_cleaning_robot_home_st_;
  BoolPub pub_cleaning_finished_;
  BoolPub pub_cleaning_active_;
  BoolPub pub_cleaning_slide_command_;
  BoolPub pub_cleaning_running_;
  BoolPub pub_cleaning_carbody_located_st_;


  rclcpp::Service<backend::srv::SetInt16>::SharedPtr srv_speed_set_;  
  BoolSrv srv_cobot_mode_set_;
  BoolSrv srv_full_automatic_mode_set_;
  //Sensing robot
  BoolSrv srv_sensing_robot_home_st_set_;
  BoolSrv srv_sensing_finished_set_;
  BoolSrv srv_touch_sensing_finished_set_;
  BoolSrv srv_sensing_active_set_;
  BoolSrv srv_touch_sensing_active_set_;
  BoolSrv srv_slide_sensing_command_set_;
  BoolSrv srv_running_sensing_set_;
  BoolSrv srv_sensing_carbody_located_set_; // this for test_server, we usually use this as observer
  BoolSrv srv_slider1_go_pos_;
  rclcpp::Service<backend::srv::SetFloat32>::SharedPtr srv_slider1_set_pos_;

  //cleaning robot
  BoolSrv srv_cleaning_robot_home_st_set_;
  BoolSrv srv_cleaning_finished_set_;
  BoolSrv srv_cleaning_active_set_;
  BoolSrv srv_slide_cleaning_command_set_;
  BoolSrv srv_running_cleaning_set_;
  BoolSrv srv_cleaning_carbody_located_set_;// this for test_server, we usually use this as observer
  BoolSrv srv_slider2_go_pos_;
  rclcpp::Service<backend::srv::SetFloat32>::SharedPtr srv_slider2_set_pos_;

  rclcpp::Service<backend::srv::GetSpotWeights>::SharedPtr welding1_data_service_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_speed_;

  // void setup_subscriptions();
  void setup_services();
  void setup_publishers();

//   std::vector<uint8_t> call_welding_data1();
//   std::vector<uint8_t> welding_data1;
// enum class SpatterMode {
//     None = 0,
//     Small = 1,
//     SmallMedium = 2,
//     BigMedium = 3,
//     Big = 4
// };
//
// std::vector<SpatterMode> spatter_modes_;
// std::mutex spatter_modes_mtx_;
};
