#include "arduino_hardware_interface/arduino_hardware_interface.hpp"
#include <cmath>

namespace arduino_diff_drive{

hardware_interface::CallbackReturn ArduinoDiffDrive::on_init
    (const hardware_interface::HardwareComponentInterfaceParams & params) 
{
    if (hardware_interface::SystemInterface::on_init(params)!=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }



    // time_ = std::chrono::system_clock::now();

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device_port"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baudrate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);


    //Set up the wheels
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    return hardware_interface::CallbackReturn::SUCCESS;
    
}

hardware_interface::CallbackReturn ArduinoDiffDrive::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    try
    {
        // Set up the Arduino
        arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Failed to open %s: %s", cfg_.device.c_str(), e.what());
        arduino_.disconnect(); 
        return hardware_interface::CallbackReturn::ERROR;
    }

    arduino_.sendResetMsg();   //reset the counter/ticks on the motor driver

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoDiffDrive::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    arduino_.sendEmptyMsg();
    arduino_.setPidValues(30, 20, 0, 100);

    arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);
    l_wheel_.pos = l_wheel_.calcEncAngle();  
    r_wheel_.pos = r_wheel_.calcEncAngle();
    l_wheel_.vel = 0;
    r_wheel_.vel = 0;

    set_command("left_rear_joint/velocity", 0.0);
    set_command("right_rear_joint/velocity", 0.0);

    // set_state("base_left_wheel_joint/velocity", 0.0);
    // set_state("base_right_wheel_joint/velocity", 0.0);
    // set_state("base_left_wheel_joint/position", 0.0);
    // set_state("base_right_wheel_joint/position", 0.0);

    // driver_->activateWithVelocityMode(left_motor_id_);
    // driver_->activateWithVelocityMode(right_motor_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
} 

hardware_interface::CallbackReturn ArduinoDiffDrive::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_INFO(logger_, "Stopping Controller...");
    arduino_.setMotorValues(0, 0);

    return hardware_interface::CallbackReturn::SUCCESS;
} 

hardware_interface::CallbackReturn ArduinoDiffDrive::on_cleanup
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    arduino_.disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoDiffDrive::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    // // Calculate time delta
    // auto curr_time = std::chrono::system_clock::now();
    // std::chrono::duration<double> time_diff = curr_time - time_;
    // double deltaSeconds = time_diff.count();
    // time_ = curr_time;

    double deltaSeconds = period.seconds();

    if (deltaSeconds <= 0.0) return hardware_interface::return_type::OK;
    
    if (!arduino_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

    double pos_prev = l_wheel_.pos;
    l_wheel_.pos = l_wheel_.calcEncAngle();
    l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = r_wheel_.pos;
    r_wheel_.pos = r_wheel_.calcEncAngle();
    r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

    set_state("left_rear_joint/velocity", l_wheel_.vel);
    set_state("right_rear_joint/velocity", r_wheel_.vel);
    set_state("left_rear_joint/position", l_wheel_.pos);
    set_state("right_rear_joint/position", r_wheel_.pos);

    return hardware_interface::return_type::OK;
} 

hardware_interface::return_type ArduinoDiffDrive::write
    (const rclcpp::Time & time, const rclcpp::Duration & period) 
{
    (void)time;
    (void)period;

    if (!arduino_.connected())
    {
        return hardware_interface::return_type::ERROR;
    }

    double l_wheel_cmd = get_command("left_rear_joint/velocity"); //rad/s
    double r_wheel_cmd = get_command("right_rear_joint/velocity"); //rad/s

    if (!std::isfinite(l_wheel_cmd)) l_wheel_cmd = 0.0;
    if (!std::isfinite(r_wheel_cmd)) r_wheel_cmd = 0.0;

    l_wheel_.cmd = l_wheel_cmd / l_wheel_.rads_per_count / cfg_.loop_rate; //counts/PID_loop
    r_wheel_.cmd = r_wheel_cmd / r_wheel_.rads_per_count / cfg_.loop_rate; //counts/PID_loop

    arduino_.setMotorValues(std::lround(l_wheel_.cmd), std::lround(r_wheel_.cmd));

    RCLCPP_INFO(logger_, "write: L=%.3f R=%.3f rad/s", l_wheel_cmd, r_wheel_cmd);

    return hardware_interface::return_type::OK;
}

} //namespace arduino_diff_drive

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arduino_diff_drive::ArduinoDiffDrive, hardware_interface::SystemInterface)

