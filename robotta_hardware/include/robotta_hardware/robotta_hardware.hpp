
#ifndef __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_H__
#define __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_H__

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>

#include "robotta_hardware/robotta_serial_port.hpp"
#include "robotta_hardware/robotta_hardware_compiler.h"

namespace robotta
{
    namespace hardware
    {
        // enum class DeviceCommand : uint8_t {
        //     MotorSetDuty = 0x01,
        //     MotorBrake   = 0x02,
        //     MotorStop    = 0x03,
        // };

        // enum class DeviceMotorDirection : uint8_t {
        //     None    = 0,
        //     Forward = 1,
        //     Reverse = 2,
        // };
        
        class RobottaHardware
            : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
        {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RobottaHardware)

            ROBOTTA_HARDWARE_PUBLIC
            virtual hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual hardware_interface::return_type start() override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual hardware_interface::return_type stop() override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual hardware_interface::return_type read() override;
            
            ROBOTTA_HARDWARE_PUBLIC
            virtual hardware_interface::return_type write() override;

        private:
            std::vector<uint8_t> motor_ids_;
            std::vector<double> position_states_;
            std::vector<double> velocity_states_;
            std::vector<double> velocity_commands_;
            std::vector<double> velocity_commands_saved_;
            std::shared_ptr<RobottaSerialPort> serial_port_;
            std::string serial_port_name_;

        };
    }
}

#endif // __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_H__