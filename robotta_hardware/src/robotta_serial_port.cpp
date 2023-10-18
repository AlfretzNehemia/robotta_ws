#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "robotta_hardware/robotta_serial_port.hpp"

#define START_FRAME     0xABCD
// #define HDLC_FRAME_BOUNDRY_FLAG     0x7E
// #define HDLC_ESCAPE_FLAG            0x7D
// #define HDLC_ESCAPE_XOR             0x20
// #define HDLC_CRC_INIT_VALUE         0xFFFF

using namespace robotta::hardware;

RobottaSerialPort::RobottaSerialPort()
    : serial_port_(-1)

{

}

RobottaSerialPort::~RobottaSerialPort()
{
    close();
}

return_type RobottaSerialPort::open(const std::string & port_name)
{
    // if (port_name != DEFAULT_PORT) {
    //     fprintf(stderr, "Port is not set in config, using default: %s", port_name.c_str());
    //     return return_type::ERROR;
    // }

    serial_port_ = ::open(port_name.c_str(), O_RDWR| O_NOCTTY);
    // serial_port_ = ::open("/dev/ttyUSB", O_RDWR | O_NOCTTY | O_NDELAY);
    // fprintf(stderr, "Success to open serial port: %s (%d)\n", strerror(errno), errno);
    if (serial_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    memset(&tty_config, 0, sizeof(termios));
    tty_config.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    tty_config.c_iflag = IGNPAR;    // default IGNPAR IGNBRK
    tty_config.c_oflag = 0;
    tty_config.c_lflag = 0;
    tty_config.c_iflag |= IXON | IXOFF;
    tty_config.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_config.c_cc[VMIN] = 1;
    tcflush(serial_port_, TCIFLUSH);
    // tcsetattr(serial_port_, TCSANOW, &tty_config);

    /*
    if (::cfsetispeed(&tty_config, B9600) != 0 || ::cfsetospeed(&tty_config, B9600) != 0) {
        fprintf(stderr, "Failed to set serial port speed: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }
    */

    if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
        fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

return_type RobottaSerialPort::close()
{
    if (is_open()) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
    return return_type::SUCCESS;
}

return_type RobottaSerialPort::read_frames()
{
    if (!is_open()) {
        // close();
        return return_type::ERROR;
    }
    if (serial_port_ != -1) {
        // uint8_t buffer[1024];
        // int r = ::read(serial_port_, buffer, sizeof(buffer));
        // if (r > 0) {
        //     for (int i = 0; i < r; ++i) {
        //         protocol_recv(buffer[i]);
        //     }
        // }
        
        uint8_t c;
        int i = 0, r = 0;
        // fprintf(stderr, "read_framessss\n");
        while ((r = ::read(serial_port_, &c, 1)) > 0 && i++ < 1024){
                // fprintf(stderr, "value of r: %i\n", r);
                // fprintf(stderr, "read_frames\n");
                protocol_recv(c);
                // fprintf(stderr, "while looping\n");
        }

        if (r < 0 && errno != EAGAIN){
                //RCLCPP_ERROR("Reading from serial %s failed: %d", port.c_str(), r);
            fprintf(stderr, "Failed to read serial port data: %s (%d)\n", strerror(errno), r);
            return return_type::ERROR;}
    }

    return return_type::SUCCESS;
}

void RobottaSerialPort::protocol_recv (uint8_t byte) {
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
    // fprintf(stderr, "read_framessss\n");
    // uint16_t start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME) {
        //RCLCPP_INFO(this->get_logger(), "Start frame recognised");
        p = (uint8_t*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
        // fprintf(stderr, "read start frames\n");
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
            // fprintf(stderr, "read message content\n");
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
            msg.wheelR_cnt ^
            msg.wheelL_cnt ^
            msg.batVoltage);
            // msg.boardTemp ^
            // msg.cmdLed
        // fprintf(stderr, "success : ");
        if (msg.start == START_FRAME && msg.checksum == checksum) {
            // std_msgs::msg::Float64 f;
            // std::vector<SerialFeedback> f;
            float volt = (double) msg.batVoltage /100;

            wheelL_hall = msg.wheelL_cnt;
            wheelR_hall = msg.wheelR_cnt;
            velL = msg.speedL_meas;
            velR = msg.speedR_meas;
            // fprintf(stderr, "Hoverboard batt voltage: : %f\n", volt, checksum);

        } else {
            fprintf(stderr, "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;

    }

    // fprintf(stderr, "speedL_meas : %f\n", velL);
    // fprintf(stderr, "speedR_meas : %f\n", velR);
    // fprintf(stderr, "wheelL_cnt : %f\n", wheelL_hall);
    // fprintf(stderr, "wheelR_cnt : %f\n", wheelR_hall);

    prev_byte = byte;
}

//return_type RobottaSerialPort::write_frame(const uint8_t* data, size_t size)
return_type RobottaSerialPort::write_frame(const double cmd_left, const double cmd_right)
{
    if (!is_open()) {
        return return_type::ERROR;
    }
    
    // Calculate steering from difference of left and right
    // const double speed = 0.02;
    // const double steer = 0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)cmd_left;
    command.speed = (int16_t)cmd_right;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(serial_port_, (const void*)&command, sizeof(command));
    if ( rc == -1) {
        fprintf(stderr, "Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

bool RobottaSerialPort::is_open() const
{
    return serial_port_ >= 0;
}
