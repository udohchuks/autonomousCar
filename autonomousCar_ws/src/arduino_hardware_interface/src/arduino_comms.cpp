#include "arduino_hardware_interface/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void ArduinoComms::disconnect()
{
    if (serial_conn_.IsOpen())
    {
        serial_conn_.Close();
    }
}

void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::sendResetMsg()
{
    std::string response = sendMsg("r\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r");

    size_t del_pos = response.find(' ');
    if (del_pos == std::string::npos)
    {
        return;
    }

    val_1 = std::atoi(response.substr(0, del_pos).c_str());
    val_2 = std::atoi(response.substr(del_pos + 1).c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    
    // std::cerr << "[ArduinoComms] sending: m " << val_1 << " " << val_2 << "\\r" << std::endl;

    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    if (!serial_conn_.IsOpen()) return "";
    serial_conn_.FlushInputBuffer();
    serial_conn_.Write(msg_to_send);

    std::string response;

    try{
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
        std::cerr << "Arduino read time out" << std::endl;
    }


    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}