/******************************
 *  SOM-O Driver Board Serial Connector & Controller
 *  Author : Theppasith Nisitsukcharoen
 *  Date : 17-Nov-2016
 *******************************/

#include <serial/serial.h>
#include "som_o_driver/controller.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include <geometry_msgs/Vector3.h>

#define DRIVE_CMD_SIZE 27
#define DRIVE_CMD_RES_SIZE 17
#define ENDING_OFFSET 4

//Debugging purpose
struct HexCharStruct
{
  unsigned char c;
  HexCharStruct(unsigned char _c) : c(_c) { }
};

inline std::ostream& operator<<(std::ostream& o, const HexCharStruct& hs)
{
  return (o << std::hex << (int)hs.c);
}

inline HexCharStruct hex(unsigned char _c)
{
  return HexCharStruct(_c);
}

namespace som_o{

    std::vector<unsigned char> stringToUnsignedCharArray(std::string input){
        // DO ESCAPE :
        int len = input.length();
        std::vector<unsigned char> hexVector;
        for(int i=1; i< len-4; i+=2)
        {
            std::string byte = input.substr(i,2);
            char chr = (char) (int)strtol(byte.c_str(), NULL, 16);
            hexVector.push_back(chr);
        }
        return hexVector;
    }

    std::string UnsignedCharArrayToString(unsigned char *input , int size){

    }

    Controller::Controller(char const *port , int baud){
        port_name_ = port;
        baud_ = baud;
        for(int i = 0 ; i< BUFFERSIZE ; i++){
            buff[i] = 0;
        }
    }

    Controller::~Controller(){
      serial_->close();
    }

    int Controller::read_vel_command(){
      serial_->read(buff,BUFFERSIZE);
      size_t ptr = 0;
      while(!(   buff[ptr+0] == ':' &&
                 buff[ptr+1] == '1' &&
                 buff[ptr+2] == '1' &&
                 buff[ptr+3] == '1'&&
                 buff[ptr+4] == '0'))
      {
          ptr++;
          if(ptr >= BUFFERSIZE)return -1;
      }

      // Found Valid Packet !
      // Create a Copy of it to check validity
      unsigned char received_packet[DRIVE_CMD_RES_SIZE-ENDING_OFFSET];
      for(int i = 0 ; i < DRIVE_CMD_RES_SIZE-ENDING_OFFSET ; i++){
          received_packet[i] = buff[ptr+i];
      }
      std::string received_string((char*) received_packet);
      std::vector<unsigned char> hexReceived;
      hexReceived = stringToUnsignedCharArray(received_string);

      //std::cout << calculateChecksum(hexReceived,6) <<std::endl;

    }

    void Controller::connect(){
      if (!serial_) serial_ = new serial::Serial();

      for (int tries = 0; tries < 5; tries++) {
        try {
          serial_ = new serial::Serial(port_name_, baud_, serial::Timeout::simpleTimeout(1000));
        } catch (serial::IOException) {
          std::cout << "Unable to open port " << std::endl;
        }

        if (serial_->isOpen()) {
          std::cout << "Successfully Connected to Driver Board" <<std::endl;
          connected_ = true;
          return;
        } else {
          connected_ = false;
          ROS_INFO("Bad Connection with serial port Error %s",port_name_);
        }
      }
      ROS_INFO("Driver Board is not responding.");
    }

    void Controller::write_velocity(int speed){

        // Header
        std::string header =  "11100018000204";

        // Low byte (set velocity) & High byte (set direction)
        std::string low_byte = "00C8"; // SpeedToStringHex(speed)
        std::string high_byte = "0000";

        std::string velocity = low_byte + high_byte;

        // CheckSum Calculating(header+low_byte+high_byte)
        std::string checksum = "F9";

        std::string command = ":" + header + velocity + checksum ;
        unsigned char cmd_buffer[DRIVE_CMD_SIZE];
        strcpy((char*)cmd_buffer,command.c_str());

        // Insert Carriage Return and Line Ending
        cmd_buffer[DRIVE_CMD_SIZE-2]  = (unsigned char)0x0D;
        cmd_buffer[DRIVE_CMD_SIZE-1]  = (unsigned char)0x0A;

        // Send Command
        serial_->write(cmd_buffer,DRIVE_CMD_SIZE);
    }


    void Controller::read(){
        serial_->read(buff,BUFFERSIZE);
    }

    int16_t Controller::calculateChecksum(std::vector<unsigned char> buffer, int iterate){
        int16_t sum = 0;
        // Sum All Packet
        for(unsigned int i = 0 ; i < iterate ; i++ ){
            sum += (int)buffer[i];
        }
        // Inverting Bits
            sum = ~sum;
        // Plus 1 -> 2's Compliment
            sum +=1;

        return sum;
    }
}
