#include "demonstrator_tree/behavior_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv){

auto pkg_path = ament_index_cpp::get_package_share_directory("demonstrator_tree");

auto cfg = ConfigLoader::load_file( pkg_path + "/config/parameters.yaml");
rclcpp::init(argc,argv);    
auto subNode = std::make_shared<DemostratorTree::MagicianSubNode>("home_check_node",cfg);
auto clientOpcUa  = std::make_shared<DemostratorTree::MagicianOpcUA>("opcua_safe_transfer_node");
auto clientNode = std::make_shared<DemostratorTree::MagicianClientNode>("homing_node",cfg,subNode);

rclcpp::executors::MultiThreadedExecutor exe;
exe.add_node(subNode);

rclcpp::executors::MultiThreadedExecutor exe_m;
exe_m.add_node(clientOpcUa);
exe_m.add_node(clientNode);


std::thread spin_subs([&exe](){
        
        while (rclcpp::ok()){
        exe.spin_some();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        }


        });


std::thread spin_client_requests([&exe_m](){
        while (rclcpp::ok()) {
        exe_m.spin_some();
        }
        });

BT::BehaviorTreeFactory factory;
factory.registerSimpleAction("IsRobotAtHome", [&](BT::TreeNode&){return subNode->CheckPos();});
factory.registerSimpleAction("CallHoming",[&](BT::TreeNode&){return clientNode->HomingCall();});
factory.registerSimpleAction("CallOpcUaHomeBothActive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::both_home>(true);});
factory.registerSimpleAction("WaitOperator",[&](BT::TreeNode&){ return subNode->CheckMode();});
factory.registerSimpleAction("IsHumanRobotMode",[&](BT::TreeNode&){return subNode->IsCobotMode();});
factory.registerSimpleAction("IsFullAutomaticMode",[&](BT::TreeNode&){return subNode->IsAutomaticMode();});
factory.registerSimpleAction("WaitingForCarLocatedCellSensing",[&](BT::TreeNode&){return subNode->CarBodyLocation("sensing_cobot");});
factory.registerSimpleAction("CallOpcUaActiveSensing",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_activation>(true);});


factory.registerSimpleAction("CallCobotOperationSensing",[&](BT::TreeNode&){return clientNode->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_operation>(true);});

factory.registerSimpleAction("CallOpcUaDeactiveSensing",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_activation>(false);});

factory.registerSimpleAction("WaitingOpcUaSensingDeactive",[&](BT::TreeNode&){return subNode->DeactivationMode("sensing_cobot");});
factory.registerSimpleAction("CallOpcUaFinishedSensingActive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_finished>(true);});

factory.registerSimpleAction("WaitingForCarLocatedCellCleaning",[&](BT::TreeNode&){return subNode->CarBodyLocation("cleaning_cobot");});


factory.registerSimpleAction("CallOpcUaHomeSensing",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_home>(true);});

factory.registerSimpleAction("CallOpcUaActiveCleaning",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_activation>(true);});


factory.registerSimpleAction("CallCobotOperationCleaning",[&](BT::TreeNode&){return clientNode->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_operation>(true);});

factory.registerSimpleAction("CallOpcUaDeactiveCleaning",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_activation>(false);});

factory.registerSimpleAction("WaitingOpcUaCleaningDeactive",[&](BT::TreeNode&){return subNode->DeactivationMode("cleaning_cobot");});

factory.registerSimpleAction("CallOpcUaHomeCleaning",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_home>(true);});

factory.registerSimpleAction("CallOpcUaFinishedCleaningActive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_finished>(true);});


factory.registerSimpleAction("CallOpcUaAutomaticModeDeactive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::automatic_mod>(false);});


factory.registerSimpleAction("CallOpcUaFinishedSensingDeactive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::sensing_finished>(false);});


factory.registerSimpleAction("CallOpcUaFinishedCleaningDeactive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::cleaning_finished>(false);});


factory.registerSimpleAction("CallOpcUaHomeBothDeactive",[&](BT::TreeNode&){return clientOpcUa->ExecuteServiceByName<DemostratorTree::TargetOpcUaClient::both_home>(false);});

auto tree =factory.createTreeFromFile( pkg_path + "/config/bt_tree.xml");
tree.tickWhileRunning();

rclcpp::shutdown();
if(spin_client_requests.joinable() || spin_subs.joinable()){
    spin_client_requests.join();
    spin_subs.join();
}

return 0;
}
