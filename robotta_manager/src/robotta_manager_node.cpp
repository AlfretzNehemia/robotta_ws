// #include <algorithm>
// #include <chrono>
// #include <memory>
// #include <string>
// #include <thread>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
// #include "robotta_hardware/robotta_hardware.hpp"
// #include "real_tools/thread_priority.hpp"

// const int DEFAULT_UPDATE_RATE = 100;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Setup the controller manager node
    std::string controller_manager_node_name = "controller_manager";
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    //std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto controller_manager_node = std::make_shared<controller_manager::ControllerManager>(executor, controller_manager_node_name);

    std::thread cm_thread([controller_manager_node]() {
    // load controller_manager update  parameter
    int update_rate = 100;
    if (!controller_manager_node->get_parameter("update_rate", update_rate)) {
        // RCLCPP_WARN(controller_manager_node->get_logger(), "'update_rate' parameter not set, using default value.");
        throw std::run_error("update_rate parameter not existing or empty");
    }
    RCLCPP_INFO(controller_manager_node->get_logger(), "update rate is %d Hz", update_rate);

    while (rclcpp::ok()) {
        std::chrono::system_::_point begin = std::chrono::system_clock::now();
        controller_manager_node->read();
        // RCLCPP_INFO(controller_manager_node->get_logger(), "Node read");
        controller_manager_node->update();
        // RCLCPP_INFO(controller_manager_node->get_logger(), "Node update");
        controller_manager_node->write();
        // RCLCPP_INFO(controller_manager_node->get_logger(), "Node write");
        std::chrono::system_clock::_point end = std::chrono::system_clock::now();
        std::this_thread::sleep_for(
            std::max(
                std::chrono::nanoseconds(0),
                std::chrono::nanoseconds(1000000000 / update_rate) -
                std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)));
    }
    });

    // Load the controllers
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    controller_manager_node->load_controller("joint_state_controller", "joint_state_controller/JointStateController");
    controller_manager_node->load_controller("robotta_drive_controller", "robotta_controller/RobottaDriveController");
    controller_manager_node->configure_controller("joint_state_controller");
    controller_manager_node->configure_controller("robotta_drive_controller");

    start_controllers.push_back("joint_state_controller");
    start_controllers.push_back("robotta_drive_controller");
    controller_manager_node->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);

    // Run the node(s)
    executor->add_node(controller_manager_node);
    executor->spin();
    cm_thread.join();
    // Exit
    rclcpp::shutdown();
    return 0;
}