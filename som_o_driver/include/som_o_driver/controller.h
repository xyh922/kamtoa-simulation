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

#define BUFFERSIZE 64
#define COMMAND_SIZE 200
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
          serial::Serial   *serial_;          // Serial port from Serial Lib
          const char       *port_name_;       // Port name for connection
          int               baud_;            // Baud Rate for serial connection
          bool              connected_;       // Flag for Connection Status
          ros::NodeHandle   nh_;              // ROS Node Handle
          uint8_t           buff[BUFFERSIZE]; // Buffer for receiving bytes
          unsigned char     cmd[COMMAND_SIZE];// Buffer for command
          int               enc_r;            // Encoder Ticks - right wheel 
          int               enc_l;            // Encoder Ticks - left wheel
      public:
          // Constructor
          Controller (const char *port, int baud);
          ~Controller();
          // Basic Serial Function
          void  read();
          void  connect();
          void  sendCommand(int cnt);

          void  stop();

          // Status Getters
          bool  is_connected() { return connected_; }

          // Prepare packet in commend sender buffer
          int   setVelCmdR(int speed);
          int   setVelCmdL(int speed);
          int   readVelCmd();

          int   setEncRead();
          int   readEnc();
          int   setVelRead();
          int   readVel();
          int   setEncVelRead();
          int   readEncVel();

  };
}
