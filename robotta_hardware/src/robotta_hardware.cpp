
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>


#include "robotta_hardware/robotta_hardware.hpp"

PLUGINLIB_EXPORT_CLASS(
    robotta::hardware::RobottaHardware,
    hardware_interface::SystemInterface
)   

using namespace robotta::hardware;

hardware_interface::return_type RobottaHardware::configure(const hardware_interface::HardwareInfo & system_info)
{

    if (configure_default(system_info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
 
    // battVoltage_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
    // board_temperatures_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
    // cfg_.device = info_.hardware_parameters["serial_port_device"];
    // cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    serial_port_name_ = info_.hardware_parameters["serial_port"];
    motor_ids_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    // TODO : Cara configure dan masukin data batteryVolt dan board temp
    // for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    //     if (sensor.name["battVoltgae_"].empty()) {
    //         RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Motor id not defined for join %s", joint.name.c_str());
    //         return hardware_interface::return_type::ERROR;
    //     }

    //     if (sensor.name["board_temperatures_"].empty()) {
    //         RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Motor id not defined for join %s", joint.name.c_str());
    //         return hardware_interface::return_type::ERROR;
    //     }
    // } 

    for (hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("RobottaHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }
    }
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    // serial_port_->open(cfg_.device);
    // serial_port_ = std::make_shared<RobottaSerialPort>();
    // if (serial_port_->open(cfg_.device) != return_type::SUCCESS) {
    //     RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware failed to open serial port");
    //     return hardware_interface::return_type::ERROR;
    // }

    // vel_pub_[0]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    // vel_pub_[1]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    // cmd_pub_[0]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    // cmd_pub_[1]    = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    // voltage_pub_   = rclcpp::create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RobottaHardware::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (size_t i = 0; i < info_.sensors.size(); i++) {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, "batteryVoltage", &battVoltage_[i]));
    // }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobottaHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type RobottaHardware::start()
{
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware starting ...please wait..");

    // for (auto i = 0; i <= hw_start_sec_; i++)
    // {
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(
    //     rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    // }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

    serial_port_ = std::make_shared<RobottaSerialPort>();
    if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware failed to open serial port");
        return hardware_interface::return_type::ERROR;
    }

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware System Successfully started!");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobottaHardware::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware is stopping ...");

    // for (auto i = 0; i <= hw_stop_sec_; i++)
    // {
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(
    //     rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    // }

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware stopped");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobottaHardware::read()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Reading...");
    // TODO : buat dua sistem, jika pake simulation gazebo dan jika pake robot real
    if (start() != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    serial_port_->read_frames();


    SerialFeedback f_data;
    velocity_states_[0] = 1 * (abs((double)f_data.speedL_meas) * 0.10472);
    velocity_states_[1] = 1 * (abs((double)f_data.speedR_meas) * 0.10472);

    position_states_[0] = f_data.wheelR_cnt;
    position_states_[1] = f_data.wheelL_cnt;  

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobottaHardware::write()
{
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Writing...");   
    if (start() != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    
    // Convert PID outputs in RAD/S to RPM
    double set_speed[2] = {
        velocity_commands_[0] / 0.10472,
        velocity_commands_[1] / 0.10472
    };

    // Calculate steering from difference of left and right
    const double speed = (set_speed[0] + set_speed[1])/2.0;
    const double steer = (set_speed[0] - speed)*2.0;

    // const double speed = -200.0;
    // const double steer = 0.0;
    
    serial_port_->write_frame(speed*2, steer*5);
    
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Motor successfully written!");
    return hardware_interface::return_type::OK;
}
