#include "demonstrator_tree/behavior_node.hpp"
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/timer.hpp>
#include <thread>

int main(int argc, char **argv){

auto cfg = ConfigLoader::load_file("/home/furkan/magician_ws/src/demonstrator_tree/config/parameters.yaml");
rclcpp::init(argc,argv);    
auto subNode = std::make_shared<DemostratorTree::MagicianSubNode>("home_check_node",cfg);
auto clientHoming = std::make_shared<DemostratorTree::MagicianClientNode>("homing_node",cfg);
auto clientOpcUa  = std::make_shared<DemostratorTree::MagicianOpcUA>("opcua_safe_transfer_node");

rclcpp::executors::SingleThreadedExecutor exe;
exe.add_node(subNode);

rclcpp::executors::MultiThreadedExecutor exe_m;
exe_m.add_node(clientOpcUa);
exe_m.add_node(clientHoming);



std::atomic<bool> initialization = false;

rclcpp::TimerBase::SharedPtr timer;

timer = subNode->create_wall_timer(
        std::chrono::seconds(2),
        [&](){
            if(initialization.exchange(true)){
            return;
            }
            timer->cancel();
        });


while (!initialization.load() && rclcpp::ok()){
    exe.spin_some();
    std::this_thread::sleep_for(std::chrono::seconds(2));
}


std::thread spin_thread([&exe_m](){
        while (rclcpp::ok()) {
        exe_m.spin_some();
        }
        });

BT::BehaviorTreeFactory factory;
factory.registerSimpleAction("IsRobotAtHome", [&](BT::TreeNode&){return subNode->checkPos();});
factory.registerSimpleAction("CallHoming",[&](BT::TreeNode&){return clientHoming->homingCall();});
factory.registerSimpleAction("CallOpcUI",[&](BT::TreeNode&){return clientOpcUa->OpcServiceCall();});
auto tree =factory.createTreeFromFile("/home/furkan/magician_ws/src/demonstrator_tree/config/bt_tree.xml");
tree.tickWhileRunning();

rclcpp::shutdown();
if(spin_thread.joinable()){
    spin_thread.join();
}

return 0;
}
