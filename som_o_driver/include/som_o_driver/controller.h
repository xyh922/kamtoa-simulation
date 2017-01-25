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
          int32_t           pos_r;            // Encoder Ticks - right wheel 
          int32_t           pos_l;            // Encoder Ticks - left wheel
          int32_t           vel_r;            // Velocity - right wheel 
          int32_t           vel_l;            // Velocity - left wheel
          int32_t           cur_r;            // Current - right wheel 
          int32_t           cur_l;            // Current - left wheel

      public:
          // Constructor
          Controller (const char *port, int baud);
          ~Controller();
          // Basic Serial Function
          void  read();                       // Read All Packet onto buffer[BUFFERSIZE]
          void  connect();                    // Connect to Driver board via Serial
          void  sendCommand(int cnt);         // Issue any command to the board
          // Stop Robot (Send 0 )
          void  stop();
          // Prepare packet in commend sender buffer
          int   setVelCmdR(int speed);
          int   setVelCmdL(int speed);
          int   readVelCmd();
          // Encoder Read
          int   setEncRead(char side);
          int   readEnc_L();
          int   readEnc_R();
          // Velocity Read
          int   setVelRead(char side);
          int   readVel_L();
          int   readVel_R();
          // Encoder and Velocity Read
          int   setEncVelRead(char side);
          int   readEncVel_L();
          int   readEncVel_R();
          // Encoder Velocity and Current Read 
          int   setEncVelCurRead(char side);
          int   readEncVelCur_L();
          int   readEncVelCur_R();
          // Buffer Checksum Validation by index 
          bool  buff_is_valid(int size);
          // Getters
          bool  is_connected() { return connected_; }
          int   getTickR(){return pos_r;}
          int   getTickL(){return pos_l;}
          int   getVelR(){return vel_r;}
          int   getVelL(){return vel_l;}

  };
}
