#include "backend/ros_bridge.hpp"
#include "backend/naming.hpp"
#include <cmath>
#include <cstdint>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <std_msgs/msg/detail/u_int8__struct.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <string>

RosBridge::RosBridge(const UaConfig& cfg)
: Node("ros2_opcua_bridge"), cfg_{cfg} {
  ua_ = std::make_shared<UaClient>();
  ua_->connect(cfg_);
  ua_->start();

  setup_publishers();
  setup_services();
  }

    
  void RosBridge::setup_publishers(){
      
  rclcpp::QoS qos(10);
  // pub_welding_data_ =this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/weld_data", qos);
  pub_speed_ = this->create_publisher<std_msgs::msg::Int16>("/ros2_comm/speed",qos);
  pub_cobot_mode_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/mod/cobot",qos);
  pub_automatic_mode_= this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/mod/automatic",qos);
  
  pub_sensing_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/home_st",qos);
  pub_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/finished",qos);
  pub_touch_sensing_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_finished",qos);
  pub_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/sensing_active",qos);
  pub_touch_sensing_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/touch_active",qos);
  pub_sensing_slide_command_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/slide_command",qos);
  pub_sensing_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/running",qos);
  pub_sensing_carbody_located_st_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/sensing/carbody_located_status", qos);
  pub_sensing_slider_actual_pos_ =this->create_publisher<std_msgs::msg::Float32>("/ros2_comm/sensing/slider_actual_pos",qos); 


  pub_cleaning_robot_home_st_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/home_st",qos);
  pub_cleaning_finished_ = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/finished",qos);
  pub_cleaning_active_ =  this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/cleaning_active",qos);
  pub_cleaning_slide_command_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/slide_command",qos);
  pub_cleaning_running_  = this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/running",qos);
  pub_cleaning_carbody_located_st_=this->create_publisher<std_msgs::msg::Bool>("/ros2_comm/cleaning/carbody_located_status",qos);
  pub_cleaning_slider_actual_pos_ =this->create_publisher<std_msgs::msg::Float32>("/ros2_comm/cleaning/slider_actual_pos",qos);
    


  // spatter_modes_.assign(cfg_.structs.spatter1_vec.size(), SpatterMode::None);

  // for(size_t i = 0; i < cfg_.structs.spatter1_vec.size(); ++i){
  //     const auto& spatter_class = cfg_.structs.spatter1_vec[i];
  //     ua_->subscribe_bool(make_child_node(spatter_class, "Small"),
  //         [this, i](bool v){ if(v) 
  //         {
  //         std::lock_guard<std::mutex> lk(spatter_modes_mtx_);
  //         spatter_modes_[i] = SpatterMode::Small; 
  //         }
  //         });
      
      
  //     ua_->subscribe_bool(make_child_node(spatter_class, "Small_Medium"),
  //         [this, i](bool v){ if(v) 
  //         {
  //         std::lock_guard<std::mutex> lk(spatter_modes_mtx_);
  //         spatter_modes_[i] = SpatterMode::SmallMedium; 
  //         }
  //         });
      
      
  //     ua_->subscribe_bool(make_child_node(spatter_class, "Big_Medium"),
  //         [this, i](bool v){ if(v) 
  //         {
  //         std::lock_guard<std::mutex> lk(spatter_modes_mtx_);
  //         spatter_modes_[i] = SpatterMode::BigMedium;
  //         }
  //         });
      

  //     ua_->subscribe_bool(make_child_node(spatter_class, "Big"),
  //         [this, i](bool v){ if(v) 
  //         {
  //         std::lock_guard<std::mutex> lk(spatter_modes_mtx_);
  //         spatter_modes_[i] = SpatterMode::Big; 
  //         }
  //         });
  // }

// ua_->subscribe_bool(cfg_.structs.spatter1_vec[0],
//         [this](bool v){
//         std_msgs::msg::Bool msg;
//         msg.data = v;
//         pub_welding_data_->publish(msg);
//         });

  ua_->subscribe_int16(cfg_.nodes.speed, [this](int16_t v){
    std_msgs::msg::Int16 msg; 
    msg.data = v;
    pub_speed_->publish(msg);
  });
   
  ua_->subscribe_bool(make_child_node(cfg_.structs.mod_root, "COBOT"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cobot_mode_->publish(msg);
  });
 
ua_->subscribe_bool(make_child_node(cfg_.structs.mod_root,"FULLY AUTOMATIC"),[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_automatic_mode_->publish(msg);
});


  ua_->subscribe_double(make_child_node(cfg_.structs.workcell_status,"Slider_2_actual position-linear"),[this](double v){
          std_msgs::msg::Float32 msg;
          msg.data = static_cast<float>(v);
          pub_cleaning_slider_actual_pos_->publish(msg);    
          });

  //sensing
  ua_->subscribe_double(make_child_node(cfg_.structs.workcell_status,"Slider_1_actual position-linear"),[this](double v){
          std_msgs::msg::Float32 msg;
          msg.data = static_cast<float>(v);   
          pub_sensing_slider_actual_pos_->publish(msg);
          });
ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "robothome_safetransfer"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_robot_home_st_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "sensing-finised"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-finished"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "sensing-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_touch_sensing_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "slide command"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_slide_command_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "running"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_sensing_running_->publish(msg);
  });
    
    ua_->subscribe_bool(make_child_node(cfg_.structs.sensing_root, "Car_Poss_Ok"), [this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_sensing_carbody_located_st_->publish(msg);
    });



  //cleaning
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "robothome_safetransfer"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_robot_home_st_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-finished"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_finished_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-active"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_active_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "slide command"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_slide_command_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root, "running"), [this](bool v){
    std_msgs::msg::Bool msg; 
    msg.data = v;
    pub_cleaning_running_->publish(msg);
  });
  ua_->subscribe_bool(make_child_node(cfg_.structs.cleaning_root,"Car_Pos_Ok"),[this](bool v){
    std_msgs::msg::Bool msg;
    msg.data = v;
    pub_cleaning_carbody_located_st_->publish(msg);
    
    });

  
}

void RosBridge::setup_services(){

  srv_cobot_mode_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/mod/cobot_mode_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.mod_root, "COBOT"), req->data);
        res->success = true;
        res->message = std::string("COBOT set to ") + (req->data ? "true" : "false");
      });

 srv_full_automatic_mode_set_ =create_service<std_srvs::srv::SetBool>(
        "/ros2_comm/mod/full_automatic_mode_set",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
            std_srvs::srv::SetBool::Response::SharedPtr res){
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.mod_root, "FULLY AUTOMATIC"), req->data);
        res->success =true;
        res->message = std::string("Full Automatic Mod set to ") + (req->data ? "true" : "false");
        });

  //sensing service
  srv_sensing_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/safetransfer_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "robothome_safetransfer"), req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "sensing-finised"), req->data);
        res->success = true;
        res->message = std::string("sensing-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_touch_sensing_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-finished"), req->data);
        res->success = true;
        res->message = std::string("touchsensing-finished set to ") + (req->data ? "true" : "false");
      });


  srv_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "sensing-active"), req->data);
        res->success = true;
        res->message = std::string("sensing-active set to ") + (req->data ? "true" : "false");
      });

  srv_touch_sensing_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/touch_active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "touchsensing-active"), req->data);
        res->success = true;
        res->message = std::string("touchsensing-active set to ") + (req->data ? "true" : "false");
      });    


  srv_slide_sensing_command_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/slide_command_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "slide command"), req->data);
        res->success = true;
        res->message = std::string("slide_command set to ") + (req->data ? "true" : "false");
      });



  srv_running_sensing_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/sensing/running",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "running"), req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });
    
  srv_sensing_carbody_located_set_= create_service<std_srvs::srv::SetBool>(
          "/ros2_comm/sensing/carbody_located_set",
          [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
              std_srvs::srv::SetBool::Response::SharedPtr res){
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.sensing_root, "Car_Poss_Ok"), req->data);
        res->success = true;
        res->message =std::string("carbody_sensing_cell_located set to ") + (req->data ? "true" : "false");
         });
  

  //cleaning service

  srv_cleaning_robot_home_st_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/safetransfer_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "robothome_safetransfer"), req->data);
        res->success = true;
        res->message = std::string("robothome_safetransfer set to ") + (req->data ? "true" : "false");
      });

  srv_cleaning_finished_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_finished_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-finished"), req->data);
        res->success = true;
        res->message = std::string("cleaning-finished set to ") + (req->data ? "true" : "false");
      });
  

  srv_cleaning_active_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/cleaning_active_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "cleaning-active"), req->data);
        res->success = true;
        res->message = std::string("cleaning-active set to ") + (req->data ? "true" : "false");
      });

  srv_slide_cleaning_command_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/slide_command_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "slide command"), req->data);
        res->success = true;
        res->message = std::string("slide_command set to ") + (req->data ? "true" : "false");
      });

  srv_running_cleaning_set_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/cleaning/running_set",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root, "running"), req->data);
        res->success = true;
        res->message = std::string("running set to ") + (req->data ? "true" : "false");
      });
  
      
  srv_speed_set_ = create_service<backend::srv::SetInt16>(
    "/ros2_comm/speed_set",
    [this](const backend::srv::SetInt16::Request::SharedPtr req,
      backend::srv::SetInt16::Response::SharedPtr res) {
        int16_t new_speed = req->data;
        ua_->enqueue_write_int16(cfg_.nodes.speed, new_speed);
        res->success = true;
        res->message = "Speed updated";
      });

  srv_slider1_go_pos_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/slider1/go_pos",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res){
      ua_->enqueue_write_double(cfg_.nodes.slider1_go, req->data);
      res->success = true;
      res->message = std::string("running set to") + (req->data ? "true" : "false");
    });
  
  srv_slider2_go_pos_ = create_service<std_srvs::srv::SetBool>(
    "/ros2_comm/slider2/go_pos",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res){
      ua_->enqueue_write_double(cfg_.nodes.slider2_go, req->data);
      res->success = true;
      res->message = std::string("running set to") + (req->data ? "true" : "false");
      });

  srv_slider1_set_pos_ = create_service<backend::srv::SetFloat32>(
    "/ros2_comm/slider1/set_pos",
    [this](const backend::srv::SetFloat32::Request::SharedPtr req,
      backend::srv::SetFloat32::Response::SharedPtr res){
      float new_pos = req->data;
      ua_->enqueue_write_double(make_child_node(cfg_.structs.workcell_status, "Slider_1_actual position-linear"), new_pos);
      res->success = true;
      res->message = "slider1 pos update";
    });
    
      srv_slider2_set_pos_ = create_service<backend::srv::SetFloat32>(
    "/ros2_comm/slider2/set_pos",
    [this](const backend::srv::SetFloat32::Request::SharedPtr req,
      backend::srv::SetFloat32::Response::SharedPtr res){
      float new_pos = req->data;
      ua_->enqueue_write_double(make_child_node(cfg_.structs.workcell_status, "Slider_2_actual position-linear"), new_pos);
      res->success = true;
      res->message = "slider2 pos update";
    });
    
      srv_cleaning_carbody_located_set_=create_service<std_srvs::srv::SetBool>(
        "/ros2_comm/cleaning/carbody_located_set", 
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res){
        ua_->enqueue_write_bool(make_child_node(cfg_.structs.cleaning_root,"Car_Pos_Ok"), req->data);
        res->success = true;
        res->message =std::string("carbody_cleaning_cell_located set to ") + (req->data ? "true" : "false");

              });

      // welding1_data_service_ = create_service<backend::srv::GetSpotWeights>(
      // "/ros2_comm/welding_data1",
      // [this](const backend::srv::GetSpotWeights::Request::SharedPtr,
      //   backend::srv::GetSpotWeights::Response::SharedPtr res){
      //  res->spots = call_welding_data1();
      //  });

}


// std::vector<uint8_t> RosBridge::call_welding_data1(){
//     welding_data1.clear();
//     std::lock_guard<std::mutex> lk(spatter_modes_mtx_);
//     for(size_t i = 0; i < spatter_modes_.size(); ++i){
//         switch(spatter_modes_[i]){
//             case SpatterMode::Small:      welding_data1.push_back(1); break;
//             case SpatterMode::SmallMedium:welding_data1.push_back(2); break;
//             case SpatterMode::BigMedium:  welding_data1.push_back(3); break;
//             case SpatterMode::Big:        welding_data1.push_back(4); break;
//             default:                      welding_data1.push_back(0); break;
//         }
//     }
//     return std::move(welding_data1);
// }

// std::vector<uint16_t> RosBridge::call_welding_data1(){

// for(const auto& spatter_class : cfg_.structs.spatter1_vec){

//     ua_->subscribe_bool(
//     make_child_node(spatter_class, "Small"),
//     [this](bool v)
//     {
//         if(v)
//             spatter_mode_ = SpatterMode::Small;
//     });

//     ua_->subscribe_bool(
//     make_child_node(spatter_class, "Small_Medium"),
//     [this](bool v)
//     {
//         if(v)
//             spatter_mode_ = SpatterMode::SmallMedium;
//     });

//     ua_->subscribe_bool(
//     make_child_node(spatter_class, "Big_Medium"),
//     [this](bool v)
//     {
//         if(v)
//             spatter_mode_ = SpatterMode::BigMedium;
//     });

//     ua_->subscribe_bool(
//     make_child_node(spatter_class, "Big"),
//     [this](bool v)
//     {
//         if(v)
//             spatter_mode_ = SpatterMode::Big;
//     });
   
    
// switch(spatter_mode_.load())
//     {
//     case SpatterMode::Small:
//         welding_data1.push_back(1); 
//         break;

//     case SpatterMode::SmallMedium:
//         welding_data1.push_back(2);
//         break;

//     case SpatterMode::BigMedium:
//         welding_data1.push_back(3);
//         break;

//     case SpatterMode::Big:
//         welding_data1.push_back(4);
//         break;

//     default:
//         welding_data1.push_back(0);
//         break;
//     }

// }
 

// return  std::move(welding_data1);


// }
