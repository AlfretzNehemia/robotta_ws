#ifndef ROBOTTA_CONFIG_H
#define ROBOTTA_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 100;
  std::string device = "/dev/ttyUSB1";
  std::string baud_rate = "B115200";
//   std::string cs8 = "CS8";
//   std::string clocal = "CLOCAL";
//   std::string cread = "CREAD";
//   int timeout = 1000;
//   int enc_counts_per_rev = 1920;
};


#endif // ROBOTTA_CONFIG_H