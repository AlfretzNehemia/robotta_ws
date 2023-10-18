
#ifndef __ROBOTTA_HARDWARE__ROBOTTA_SERIAL_PORT_H__
#define __ROBOTTA_HARDWARE__ROBOTTA_SERIAL_PORT_H__

#include <string>
#include <vector>

#define ROBOTTA_SERIAL_BUFFER_MAX_SIZE           200
#define ROBOTTA_SERIAL_SERIAL_FRAME_MAX_SIZE     100


namespace robotta
{
    namespace hardware
    {
        enum class return_type : std::uint8_t
        {
            SUCCESS = 0,
            ERROR = 1
        };

        // struct SerialHdlcFrame
        // {
        //     uint8_t data[ROBOTTA_SERIAL_SERIAL_FRAME_MAX_SIZE];
        //     size_t length;
        // };

        typedef struct {
            uint16_t start;
            int16_t  steer;
            int16_t  speed;
            uint16_t checksum;
            
        } SerialCommand;

        typedef struct {
            uint16_t start;
            int16_t  cmd1;
            int16_t  cmd2;
            int16_t  speedR_meas;
            int16_t  speedL_meas;
            int16_t  wheelR_cnt;
            int16_t  wheelL_cnt; 
            int16_t  batVoltage;
            // int16_t  boardTemp;
            // uint16_t cmdLed;
            uint16_t checksum;
        } SerialFeedback;

        class RobottaSerialPort
        {
        public:
            RobottaSerialPort();
            ~RobottaSerialPort();
            
            return_type open(const std::string & port_name);
            return_type close();
            // SerialFeedback msg, prev_msg;
            // return_type read_frames(std::vector<SerialFeedback>& frames);
            return_type read_frames();
            // return_type write_frame(const uint8_t* data, size_t size);
            // return_type write_frame();
            return_type write_frame(const double speed, const double steer);
            bool is_open() const;

            double wheelL_hall;
            double wheelR_hall;
            double velL;
            double velR;

        protected:
            // void encode_byte(uint8_t data);
            // void decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames);
            // uint16_t crc_update(uint16_t crc, uint8_t data);

        private:
            void protocol_recv(uint8_t byte);
            int msg_len = 0;
            uint8_t prev_byte = 0;
            uint16_t start_frame = 0;
            uint8_t* p;
            SerialFeedback msg, prev_msg;

            int serial_port_;
            // uint8_t rx_buffer_[ROBOTTA_SERIAL_BUFFER_MAX_SIZE];
            // uint8_t rx_frame_buffer_[ROBOTTA_SERIAL_SERIAL_FRAME_MAX_SIZE];
            // size_t rx_frame_length_;
            // uint16_t rx_frame_crc_;
            // bool rx_frame_escape_;
            // uint8_t tx_frame_buffer_[ROBOTTA_SERIAL_SERIAL_FRAME_MAX_SIZE];
            // size_t tx_frame_length_;
            // uint16_t tx_frame_crc_;

        };
    }
}

#endif // __ROBOTTA_HARDWARE__ROBOTTA_SERIAL_PORT_H__
