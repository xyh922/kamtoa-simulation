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
#include <boost/timer/timer.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <bitset>

#include <geometry_msgs/Vector3.h>

#define DRIVE_CMD_SIZE 27
#define DRIVE_CMD_RES_SIZE 17
#define ENDING_OFFSET 4
#define READ_TIMEOUT_MILLISEC 5

namespace som_o
{

    Controller::Controller(char const *port, int baud)
    {
        port_name_ = port;
        baud_ = baud;
        for (int i = 0; i < BUFFERSIZE; i++)
        {
            buff[i] = 0;
        }
    }

    Controller::~Controller()
    {
        this->stop();
        serial_->close();
    }

    void Controller::connect()
    {
        if (!serial_)
            this->serial_ = new serial::Serial();

        for (int tries = 0; tries < 5; tries++)
        {
            try
            {
                this->serial_ = new serial::Serial(port_name_, baud_, 
                serial::Timeout::simpleTimeout(10)); //30
            }
            catch (serial::IOException)
            {
                ROS_INFO("[Driver] Unable to Open Port :  %s", port_name_);
            }

            if (this->serial_->isOpen())
            {
                ROS_INFO("[Driver] Successfully connected to driver board : %s", port_name_);
                connected_ = true;
                return;
            }
            else
            {
                connected_ = false;
                ROS_INFO("[Driver] Bad Connection with serial port Error %s", port_name_);
            }
        }
        ROS_INFO("Driver Board is not responding.");
    }

    void Controller::stop(){
        this->sendCommand(this->setVelCmdL(0));
        this->readVelCmd();

        usleep(5*1000);
        this->sendCommand(this->setVelCmdR(0));
        this->readVelCmd();
    }

    void Controller::sendCommand(int cnt)
    {
        // Create container for sending command
        char tmp[200];
        int tmp_cnt = 0;

        // Header
        tmp[tmp_cnt++] = ':';

        // Calculate CHECKSUM [ Command : cmd[i] ]
        int sum = 0;
        for (int i = 0; i < cnt; i++)
        {
            sum += cmd[i];
            tmp_cnt += sprintf(tmp + tmp_cnt, "%02X", cmd[i]);
        }
        tmp_cnt += sprintf(tmp + tmp_cnt, "%02X", (0x100 - sum) & 0xff);

        // Two Ending bytes
        tmp[tmp_cnt++] = (unsigned char)0x0D;
        tmp[tmp_cnt++] = (unsigned char)0x0A;

        // Send Command
        this->serial_->write((unsigned char *)tmp, tmp_cnt); //std::cout << "setVelCmd (" << tmp_cnt << ") : " << tmp << std::endl;
    }

    int Controller::setVelCmdL(int vel)
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

    int Controller::setVelCmdR(int vel)
    {
        int cnt = 0;
        cmd[cnt++] = 0x12;
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

    int Controller::setEncRead()
    {
        int cnt = 0;
        cmd[cnt++] = 0x11;
        cmd[cnt++] = 0x04;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x02;

        return cnt;
    }

    int Controller::readEnc()
    {

    }

    int Controller::setVelRead()
    {
        int cnt = 0;
        cmd[cnt++] = 0x11;
        cmd[cnt++] = 0x04;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x02;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x02;

        return cnt;
    }

    int Controller::readVel()
    {

    }

    int Controller::setEncVelRead()
    {
        int cnt = 0;
        cmd[cnt++] = 0x11;
        cmd[cnt++] = 0x04;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x04;

        return cnt;
    }

    int Controller::readEncVel()
    {

    }

    int Controller::readVelCmd()
    {
        // Check the size of the incoming message
        size_t available = this->serial_->available();
        if (available > BUFFERSIZE) // Message is too big -> Discard this incoming package
        {
            ROS_INFO(", now flushing in an attempt to catch up.");
            this->serial_->flushInput();
            return 0;
        }
        // Read Package
        int n = this->serial_->read(buff, 17);
            std::cout << "Recieving (" << n << ") = ";
            for (int i = 0; i < n - 2; i++)
            { // n-2 for ignore line carriage
                printf("%c", (int)buff[i]);
            }
            std::cout << std::endl;
            this->serial_->flush(); //Discard following bytes
        
       
    }

    void Controller::read()
    {
        this->serial_->read(buff, BUFFERSIZE);
    }
}
