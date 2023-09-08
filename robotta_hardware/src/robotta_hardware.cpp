
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

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
        // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }

    // serial_port_->open(cfg_.device);
    // serial_port_ = std::make_shared<RobottaSerialPort>();
    // if (serial_port_->open(cfg_.device) != return_type::SUCCESS) {
    //     RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware failed to open serial port");
    //     return hardware_interface::return_type::ERROR;
    // }
    // odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    //     DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    // vel_meas_left  = node.create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    // vel_meas_right = node.create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    // vel_cmd_left   = node.create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    // vel_cmd_right  = node.create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    // voltage_       = node.create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RobottaHardware::export_state_interfaces()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (size_t i = 0; i < info_.sensors.size(); i++) {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, "batteryVoltage", &battVoltage_[i]));
    // }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobottaHardware::export_command_interfaces()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type RobottaHardware::start()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware starting ...please wait..");

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

    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware System Successfully started!");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobottaHardware::stop()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Robotta hardware is stopping ...");

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

    if (start() != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    // std_msgs::msg::Float64 f;

    serial_port_->read_frames();

    velocity_states_[0] = 1 * (abs((double)serial_port_->velL)); // data in rpm unit
    velocity_states_[1] = 1 * (abs((double)serial_port_->velR)); // data in rpm unit

    // position_states_[0] = serial_port_->wheelL_hall;
    // position_states_[1] = serial_port_->wheelR_hall;

    on_encoder_update(serial_port_->wheelL_hall, serial_port_->wheelR_hall);

    // f.data = (double)serial_port_->velL;
    // vel_meas_left->publish(f);

    // f.data = (double)serial_port_->velR;
    // vel_meas_right->publish(f);

    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Posisi Kiri: %f", position_states_[0]);
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Posisi Kanan: %f", position_states_[1]);
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Kecepatan Kiri: %f", velocity_states_[0]);
    RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Kecepatan Kanan: %f", velocity_states_[1]);
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobottaHardware::write()
{
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Writing...");   
    if (start() != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    
    // Convert command velovity from RAD/S to RPM
    double set_cmd_left = velocity_commands_[0] / 0.10472;
    double set_cmd_right = velocity_commands_[1] / 0.10472;

    // std_msgs::msg::Float64 m;

    // m.data = (double)set_cmd_left;
    // vel_cmd_left->publish(m);

    // m.data = (double)set_cmd_right;
    // vel_cmd_right->publish(m);


    // Calculate steering from difference of left and right
    // set_speed[0] = left wheel
    // set_speed[1] = right wheel
    // const double speed = (set_speed[0] + set_speed[1])/2.0;
    // const double steer = (set_speed[0] - speed)*2.0;

    // const double set_cmd_left = -200.0;
    // const double set_cmd_right = 0.0;
    
    serial_port_->write_frame(set_cmd_left, set_cmd_right);
    
    // RCLCPP_INFO(rclcpp::get_logger("RobottaHardware"), "Motor successfully written!");
    return hardware_interface::return_type::OK;
}

void RobottaHardware::on_encoder_update(int16_t right, int16_t left){
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddenly lost
    // This section accumulates ticks even if board shuts down and is restarted
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    // auto clock = std::make_shared<rclcpp::Clock>();
    rclcpp::Clock clock;
    auto now = clock.now();

    // auto now = clock->now();
    // If there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restart
    // (the board seems to often report 1-3 ticks on startup instead of zero)
    // Reset the last read ticks to the startup values
    if ((now - last_read).seconds() > 0.2
		&& std::abs(posL) < 5 && std::abs(posR) < 5) {
        lastPosL = posL;
        lastPosR = posR;
    }
    double posLDiff = 0;
    double posRDiff = 0;

    // If node is just starting, keep odom at zeros
    if (nodeStartFlag) {
        nodeStartFlag = false;
    } else {
        posLDiff = posL - lastPosL;
        posRDiff = posR - lastPosR;
    }

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;

    // Convert position in accumulated ticks to position in radians
    position_states_[1] = 2.0*M_PI * lastPubPosL / (double)TICKS_PER_ROTATION;
    position_states_[0] = 2.0*M_PI * lastPubPosR / (double)TICKS_PER_ROTATION;

    // fprintf(stderr, "posL : %f\n", position_states_[0]);
    // fprintf(stderr, "posR : %f\n", position_states_[1]);
    // pos_pub[0] = rclcpp::create_publisher<Float64>(position_states_[0]);
    // pos_pub[1] = rclcpp::create_publisher<Float64>(position_states_[1]);

    // pos_pub[0].publish(position_states_[0]);
    // pos_pub[1].publish(position_states_[1]);
}