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
#include <boost/algorithm/string.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <bitset>

#include <geometry_msgs/Vector3.h>

#define DRIVE_CMD_SIZE 27
#define DRIVE_CMD_RES_SIZE 17
#define ENDING_OFFSET 4

namespace som_o{

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
      unsigned char received_packet[DRIVE_CMD_RES_SIZE];
      for(int i = 0 ; i < DRIVE_CMD_RES_SIZE; i++){
          received_packet[i] = buff[ptr+i];
      }
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

    void Controller::sendCommand(int cnt)
    {
    	char tmp[200];
    	int tmp_cnt = 0;
    	tmp[tmp_cnt++] = ':';

    	int sum = 0;
    	for(int i = 0; i < cnt; i++)
    	{
    		sum += cmd[i];
    		tmp_cnt+= sprintf(tmp+tmp_cnt, "%02X", cmd[i]);
    	}
    	tmp_cnt+= sprintf(tmp+tmp_cnt, "%02X", (0x100 - sum) & 0xff);
    	tmp[tmp_cnt++] = (unsigned char)0x0D;
    	tmp[tmp_cnt++] = (unsigned char)0x0A;

    	std::cout << "setVelCmd (" << tmp_cnt << ") : " << tmp << std::endl;

      // Send Command
      serial_->write((unsigned char*)tmp,tmp_cnt);
    }


    int Controller::setVelCmd(int vel)
    {
    	int cnt = 0;
    	cmd[cnt++] = 0x11;
    	cmd[cnt++] = 0x10;
    	cmd[cnt++] = 0x00;
    	cmd[cnt++] = 0x18;
    	cmd[cnt++] = 0x00;
    	cmd[cnt++] = 0x02;
    	cmd[cnt++] = 0x04;

    	cmd[cnt++] = (vel & 0xffff) >> 8;
    	cmd[cnt++] = (vel & 0xffff) & 0xff;
    	cmd[cnt++] = (vel >> 16) >> 8;
    	cmd[cnt++] = (vel >> 16) & 0xff;

    	return cnt;
    }

    void Controller::read(){
        serial_->read(buff,BUFFERSIZE);
    }

    std::string Controller::calculateChecksum(std::vector<unsigned char> buffer, int iterate){
        int8_t sum = 0;
        // Sum All Packet
        std::cout << "========" <<std::endl;
        for(unsigned int i = 0 ; i < iterate ; i++ ){
            //std::cout << hex(buffer[i]) <<std::endl;
            sum += (int8_t)buffer[i];
        }
        // Inverting Bits
            sum = ~sum;
        // Plus 1 -> 2's Compliment
            sum +=1;

            int8_t first = (sum >> 4)& 0b00001111;
            int8_t second = sum & 0b00001111;
            int intval = first;
            char hexval[5];
            sprintf(hexval,"%0x",intval);
            std::stringstream ss;
            std::string s1;
            ss << hexval[0];
            ss>> s1;
            intval = second;
            sprintf(hexval,"%0x",intval);
            std::stringstream ss2;
            std::string s2;
            ss2 << hexval[0];
            ss2>> s2;
            std::string checkk = s1+s2;
            boost::to_upper(checkk);
            std::cout << checkk <<std::endl;

        return checkk;
    }
}
