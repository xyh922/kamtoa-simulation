/******************************
 *  [Header]SOM-O Driver Board Serial Connector & Controller
 *  Author : Theppasith Nisitsukcharoen
 *  Date : 17-Nov-2016
 *******************************/
#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
#include <stdint.h>
#include <string>
#include <serial/serial.h>
#include <ros/ros.h>

#define BUFFERSIZE 40
#define CR   0x0D
#define LF   0x0A

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif

namespace som_o{

  class Controller{
      private:
          const char       *port_name_;   // Port name for connection
          serial::Serial   *serial_;          // Serial port from Serial Lib
          int               baud_;        // Baud Rate for serial connection
          bool              connected_;   // Flag for Connection Status
          ros::NodeHandle   nh_;
          uint8_t           buff[BUFFERSIZE]; // Buffer for receiving bytes

          // Buffer for command
          unsigned char     cmd[200];

          // Read and Write (Serial)

          void write(std::string);

      public:
          Controller (const char *port, int baud);
          ~Controller();

          void read();  
          // Create connection to board
          void connect();

          // Getters
          bool is_connected() { return connected_; }

          // Prepare packet in commend sender buffer
          int setVelCmd(int speed);
          void sendCommand(int cnt);
          // Write any command to serial

          int read_vel_command();

          // Calculate Checksum
          std::string calculateChecksum(std::vector<unsigned char> buffer , int iterate);
  };
}
