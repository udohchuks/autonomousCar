#ifndef ARDUINO_DIFF_DRIVE_INTERFACE_HPP
#define ARDUINO_DIFF_DRIVE_INTERFACE_HPP

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"

#include "arduino_hardware_interface/config.h"
#include "arduino_hardware_interface/wheel.h"
#include "arduino_hardware_interface/arduino_comms.h"


namespace arduino_diff_drive{

class ArduinoDiffDrive : public hardware_interface::SystemInterface
{
public:
    //lifecycle node override
    // re-declaration and re-definition of the function on_configure (with pass by reference) which already exists in the parent class as a virtual function
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override; // also calls the construction of the system interface class
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    Config cfg_;
    ArduinoComms arduino_;

    Wheel l_wheel_;
    Wheel r_wheel_;

    rclcpp::Logger logger_ = rclcpp::get_logger("ArduinoDiffDrive");
    
    // std::chrono::time_point<std::chrono::system_clock> time_;

}; // class ArduinoDiffDrive

} // namespace arduino_diff_drive

#endif