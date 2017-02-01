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
        ROS_INFO("STOP CONTROLLER : ISSUE 0,0");
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

    bool Controller::buff_is_valid(int size){
        // Transform buff to hex by 2 char 
        int cnt = 0;
        char rec[200];
        int sum = 0;
        for(int i = 1 ; i < size - 2 ; i+=2){
            rec[cnt]    = (buff[i] <= '9' ? buff[i] - '0' : toupper(buff[i]) - 'A' + 10) << 4;
            rec[cnt]   |= buff[i+1] <= '9' ? buff[i+1] - '0' : toupper(buff[i+1]) - 'A' + 10;
            sum += rec[cnt];
            cnt++;
        }
        sum = sum & 0x00ff;
        //std::cout << "SM = " << sum <<std::endl;
        return sum==0;
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

        // Packet Header
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

    int Controller::setEncRead(char side)
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


    int Controller::setVelRead(char side)
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

    int Controller::readVel_L()
    {
        int n = this->serial_->read(buff, 20);
        std::cout << n ;
        if( buff[0] == ':' && buff_is_valid(n)){
            vel_l = 0;
            vel_l   |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            vel_l   |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;

            vel_l   |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            vel_l   |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;

            vel_l   |= (buff[7]  <= '9' ? buff[7]  - '0' : toupper(buff[7])  - 'A' + 10) << 12;
            vel_l   |= (buff[8]  <= '9' ? buff[8]  - '0' : toupper(buff[8])  - 'A' + 10) << 8;

            vel_l   |= (buff[9]  <= '9' ? buff[9]  - '0' : toupper(buff[9])  - 'A' + 10) << 4;
            vel_l   |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;

            std::cout << "[L-Velocity] = " << vel_l <<std::endl;
            this->serial_->flush();
        }
        else
        {
            std::cout << "[L-Velocity]Invalid Receive Discard !" <<std::endl;
            this->serial_->flush();
        }
    }
    int Controller::readVel_R()
    {
        int n = this->serial_->read(buff, 20);
        std::cout << n ;
        if( buff[0] == ':' && buff_is_valid(n)){
            vel_r = 0;
            vel_r   |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            vel_r   |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;

            vel_r   |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            vel_r   |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;

            vel_r   |= (buff[7]  <= '9' ? buff[7]  - '0' : toupper(buff[7])  - 'A' + 10) << 12;
            vel_r   |= (buff[8]  <= '9' ? buff[8]  - '0' : toupper(buff[8])  - 'A' + 10) << 8;

            vel_r   |= (buff[9]  <= '9' ? buff[9]  - '0' : toupper(buff[9])  - 'A' + 10) << 4;
            vel_r   |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;

            std::cout << "[R-Velocity] = " << vel_r <<std::endl;
            this->serial_->flush();
        }
        else
        {
            std::cout << "[R-Velocity]Invalid Receive Discard !" <<std::endl;
            this->serial_->flush();
        }
    }


    int Controller::readEnc_L()
    {
        int n = this->serial_->read(buff, 20);
        if( buff[0] == ':' && buff_is_valid(n)){
            pos_l = 0;
            pos_l   |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            pos_l   |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;

            pos_l   |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            pos_l   |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;

            pos_l   |= (buff[7]  <= '9' ? buff[7]  - '0' : toupper(buff[7])  - 'A' + 10) << 12;
            pos_l   |= (buff[8]  <= '9' ? buff[8]  - '0' : toupper(buff[8])  - 'A' + 10) << 8;

            pos_l   |= (buff[9]  <= '9' ? buff[9]  - '0' : toupper(buff[9])  - 'A' + 10) << 4;
            pos_l   |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;

            std::cout << "[L-Position] = " << pos_l <<std::endl;
            this->serial_->flush();
        }
        else
        {
            std::cout << "[L-Position]Invalid Receive Discard !" <<std::endl;
            this->serial_->flush();
        }
    }

    int Controller::readEnc_R()
    {
        int n = this->serial_->read(buff, 20);
        if( buff[0] == ':' && buff_is_valid(n)){
            pos_r = 0;
            pos_r   |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            pos_r   |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;

            pos_r   |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            pos_r   |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;

            pos_r   |= (buff[7]  <= '9' ? buff[7]  - '0' : toupper(buff[7])  - 'A' + 10) << 12;
            pos_r   |= (buff[8]  <= '9' ? buff[8]  - '0' : toupper(buff[8])  - 'A' + 10) << 8;

            pos_r   |= (buff[9]  <= '9' ? buff[9]  - '0' : toupper(buff[9])  - 'A' + 10) << 4;
            pos_r   |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;

            std::cout << "[R-Position] = " << pos_l <<std::endl;
            this->serial_->flush();
        }
        else
        {
            std::cout << "[R-Position]Invalid Receive Discard !" <<std::endl;
            this->serial_->flush();
        }
    }


    int Controller::setEncVelRead(char side)
    {
        int cnt = 0;

        if(side == 'l' || side == 'L')cmd[cnt++] = 0x11;
        else if(side =='r' || side == 'R')cmd[cnt++] = 0x12;

        cmd[cnt++] = 0x04;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x04;

        return cnt;
    }

    int Controller::readEncVel_L()
    {
        int n = this->serial_->read(buff, 32);
        //std::cout << "Recieving L (" << n << ") = ";
        //std::cout << buff ;
        if( buff[0] == ':' && buff_is_valid(n)){
            pos_l = 0;
            pos_l   |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            pos_l   |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;

            pos_l   |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            pos_l   |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;

            pos_l   |= (buff[7]   <= '9' ? buff[7]   - '0' : toupper(buff[7])   - 'A' + 10) << 12;
            pos_l   |= (buff[8]   <= '9' ? buff[8]   - '0' : toupper(buff[8])   - 'A' + 10) << 8;

            pos_l   |= (buff[9]   <= '9' ? buff[9]   - '0' : toupper(buff[9])   - 'A' + 10) << 4;
            pos_l   |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;
            
            vel_l = 0;
            vel_l   |= (buff[19]  <= '9' ? buff[19]  - '0' : toupper(buff[19])  - 'A' + 10) << 28;
            vel_l   |= (buff[20]  <= '9' ? buff[20]  - '0' : toupper(buff[20])  - 'A' + 10) << 24;

            vel_l   |= (buff[21]  <= '9' ? buff[21]  - '0' : toupper(buff[21])  - 'A' + 10) << 20;
            vel_l   |= (buff[22]  <= '9' ? buff[22]  - '0' : toupper(buff[22])  - 'A' + 10) << 16;

            vel_l   |= (buff[15]  <= '9' ? buff[15]  - '0' : toupper(buff[15])  - 'A' + 10) << 12;
            vel_l   |= (buff[16]  <= '9' ? buff[16]  - '0' : toupper(buff[16])  - 'A' + 10) << 8;

            vel_l   |= (buff[17]  <= '9' ? buff[17]  - '0' : toupper(buff[17])  - 'A' + 10) << 4;
            vel_l   |= (buff[18]  <= '9' ? buff[18]  - '0' : toupper(buff[18])  - 'A' + 10) << 0;

            //printf("%d , %d ", pos_l , vel_l);
            //std::cout << std::endl;
            //this->serial_->flush();
        }
        else
        {
            //std::cout << "[L Velo-Pos]Invalid Receive Discard !" <<std::endl;
            //this->serial_->flush();
        }
        
    }

    int Controller::readEncVel_R()
     {
       int n = this->serial_->read(buff, 64);
       //std::cout << "Recieving R (" << n << ") = ";
         //std::cout << buff ;
        if(buff[0] == ':' && buff_is_valid(n)){
            pos_r = 0;
            pos_r    |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
            pos_r    |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;
            
            pos_r    |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
            pos_r    |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;
            
            pos_r    |= (buff[7]   <= '9' ? buff[7]   - '0' : toupper(buff[7])   - 'A' + 10) << 12;
            pos_r    |= (buff[8]   <= '9' ? buff[8]   - '0' : toupper(buff[8])   - 'A' + 10) << 8;

            pos_r    |= (buff[9]   <= '9' ? buff[9]   - '0' : toupper(buff[9])   - 'A' + 10) << 4;
            pos_r    |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;
            
            vel_r = 0;
            vel_r    |= (buff[19]  <= '9' ? buff[19]  - '0' : toupper(buff[19])  - 'A' + 10) << 28;
            vel_r    |= (buff[20]  <= '9' ? buff[20]  - '0' : toupper(buff[20])  - 'A' + 10) << 24;

            vel_r    |= (buff[21]  <= '9' ? buff[21]  - '0' : toupper(buff[21])  - 'A' + 10) << 20;
            vel_r    |= (buff[22]  <= '9' ? buff[22]  - '0' : toupper(buff[22])  - 'A' + 10) << 16;

            vel_r    |= (buff[15]  <= '9' ? buff[15]  - '0' : toupper(buff[15])  - 'A' + 10) << 12;
            vel_r    |= (buff[16]  <= '9' ? buff[16]  - '0' : toupper(buff[16])  - 'A' + 10) << 8;

            vel_r    |= (buff[17]  <= '9' ? buff[17]  - '0' : toupper(buff[17])  - 'A' + 10) << 4;
            vel_r    |= (buff[18]  <= '9' ? buff[18]  - '0' : toupper(buff[18])  - 'A' + 10) << 0;

            //printf("%d , %d ", pos_r , vel_r);
            //std::cout << std::endl;
            //this->serial_->flush();
        }
        else
        {
           // std::cout << "[R Velo-Pos]Invalid Receive Discard !" <<std::endl;
            this->serial_->flush();
        }
        
    }

    int Controller::setEncVelCurRead(char side){
        int cnt = 0;

        if(side == 'l' || side == 'L')cmd[cnt++] = 0x11;
        else if(side =='r' || side == 'R')cmd[cnt++] = 0x12;

        cmd[cnt++] = 0x04;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x00;
        cmd[cnt++] = 0x06;

        return cnt;
    }

    int Controller::readEncVelCur_L(){
        int n = this->serial_->read(buff, 64);
        if(buff[0] == ':' && buff_is_valid(n)){
            pos_l = 0;
                pos_l    |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
                pos_l    |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;
            
                pos_l    |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
                pos_l    |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;
                
                pos_l    |= (buff[7]   <= '9' ? buff[7]   - '0' : toupper(buff[7])   - 'A' + 10) << 12;
                pos_l    |= (buff[8]   <= '9' ? buff[8]   - '0' : toupper(buff[8])   - 'A' + 10) << 8;

                pos_l    |= (buff[9]   <= '9' ? buff[9]   - '0' : toupper(buff[9])   - 'A' + 10) << 4;
                pos_l    |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;
                
            vel_l = 0;
                vel_l    |= (buff[19]  <= '9' ? buff[19]  - '0' : toupper(buff[19])  - 'A' + 10) << 28;
                vel_l    |= (buff[20]  <= '9' ? buff[20]  - '0' : toupper(buff[20])  - 'A' + 10) << 24;

                vel_l    |= (buff[21]  <= '9' ? buff[21]  - '0' : toupper(buff[21])  - 'A' + 10) << 20;
                vel_l    |= (buff[22]  <= '9' ? buff[22]  - '0' : toupper(buff[22])  - 'A' + 10) << 16;

                vel_l    |= (buff[15]  <= '9' ? buff[15]  - '0' : toupper(buff[15])  - 'A' + 10) << 12;
                vel_l    |= (buff[16]  <= '9' ? buff[16]  - '0' : toupper(buff[16])  - 'A' + 10) << 8;

                vel_l    |= (buff[17]  <= '9' ? buff[17]  - '0' : toupper(buff[17])  - 'A' + 10) << 4;
                vel_l    |= (buff[18]  <= '9' ? buff[18]  - '0' : toupper(buff[18])  - 'A' + 10) << 0;
            cur_l = 0;
                cur_l    |= (buff[27]  <= '9' ? buff[27]  - '0' : toupper(buff[27])  - 'A' + 10) << 28;
                cur_l    |= (buff[28]  <= '9' ? buff[28]  - '0' : toupper(buff[28])  - 'A' + 10) << 24;

                cur_l    |= (buff[29]  <= '9' ? buff[29]  - '0' : toupper(buff[29])  - 'A' + 10) << 20;
                cur_l    |= (buff[30]  <= '9' ? buff[30]  - '0' : toupper(buff[30])  - 'A' + 10) << 16;

                cur_l    |= (buff[23]  <= '9' ? buff[23]  - '0' : toupper(buff[23])  - 'A' + 10) << 12;
                cur_l    |= (buff[24]  <= '9' ? buff[24]  - '0' : toupper(buff[24])  - 'A' + 10) << 8;

                cur_l    |= (buff[25]  <= '9' ? buff[25]  - '0' : toupper(buff[25])  - 'A' + 10) << 4;
                cur_l    |= (buff[26]  <= '9' ? buff[26]  - '0' : toupper(buff[26])  - 'A' + 10) << 0;
        }
        
    }

    int Controller::readEncVelCur_R(){
        int n = this->serial_->read(buff, 64);
        if(buff[0] == ':' && buff_is_valid(n)){
            pos_r = 0;
                pos_r    |= (buff[11]  <= '9' ? buff[11]  - '0' : toupper(buff[11])  - 'A' + 10) << 28;
                pos_r    |= (buff[12]  <= '9' ? buff[12]  - '0' : toupper(buff[12])  - 'A' + 10) << 24;
                
                pos_r    |= (buff[13]  <= '9' ? buff[13]  - '0' : toupper(buff[13])  - 'A' + 10) << 20;
                pos_r    |= (buff[14]  <= '9' ? buff[14]  - '0' : toupper(buff[14])  - 'A' + 10) << 16;
                
                pos_r    |= (buff[7]   <= '9' ? buff[7]   - '0' : toupper(buff[7])   - 'A' + 10) << 12;
                pos_r    |= (buff[8]   <= '9' ? buff[8]   - '0' : toupper(buff[8])   - 'A' + 10) << 8;

                pos_r    |= (buff[9]   <= '9' ? buff[9]   - '0' : toupper(buff[9])   - 'A' + 10) << 4;
                pos_r    |= (buff[10]  <= '9' ? buff[10]  - '0' : toupper(buff[10])  - 'A' + 10) << 0;
                
            vel_r = 0;
                vel_r    |= (buff[19]  <= '9' ? buff[19]  - '0' : toupper(buff[19])  - 'A' + 10) << 28;
                vel_r    |= (buff[20]  <= '9' ? buff[20]  - '0' : toupper(buff[20])  - 'A' + 10) << 24;

                vel_r    |= (buff[21]  <= '9' ? buff[21]  - '0' : toupper(buff[21])  - 'A' + 10) << 20;
                vel_r    |= (buff[22]  <= '9' ? buff[22]  - '0' : toupper(buff[22])  - 'A' + 10) << 16;

                vel_r    |= (buff[15]  <= '9' ? buff[15]  - '0' : toupper(buff[15])  - 'A' + 10) << 12;
                vel_r    |= (buff[16]  <= '9' ? buff[16]  - '0' : toupper(buff[16])  - 'A' + 10) << 8;

                vel_r    |= (buff[17]  <= '9' ? buff[17]  - '0' : toupper(buff[17])  - 'A' + 10) << 4;
                vel_r    |= (buff[18]  <= '9' ? buff[18]  - '0' : toupper(buff[18])  - 'A' + 10) << 0;
            cur_r = 0;
                cur_r    |= (buff[27]  <= '9' ? buff[27]  - '0' : toupper(buff[27])  - 'A' + 10) << 28;
                cur_r    |= (buff[28]  <= '9' ? buff[28]  - '0' : toupper(buff[28])  - 'A' + 10) << 24;

                cur_r    |= (buff[29]  <= '9' ? buff[29]  - '0' : toupper(buff[29])  - 'A' + 10) << 20;
                cur_r    |= (buff[30]  <= '9' ? buff[30]  - '0' : toupper(buff[30])  - 'A' + 10) << 16;

                cur_r    |= (buff[23]  <= '9' ? buff[23]  - '0' : toupper(buff[23])  - 'A' + 10) << 12;
                cur_r    |= (buff[24]  <= '9' ? buff[24]  - '0' : toupper(buff[24])  - 'A' + 10) << 8;

                cur_r    |= (buff[25]  <= '9' ? buff[25]  - '0' : toupper(buff[25])  - 'A' + 10) << 4;
                cur_r    |= (buff[26]  <= '9' ? buff[26]  - '0' : toupper(buff[26])  - 'A' + 10) << 0;
        }
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
        // std::cout << "Recieving (" << n << ") = ";
        // for (int i = 0; i < n - 2; i++)
        // { // n-2 for ignore line carriage
        //     printf("%c", (int)buff[i]);
        // }
        // std::cout << std::endl;
        this->serial_->flush(); //Discard following bytes      
    }

    void Controller::read()
    {
        this->serial_->read(buff, BUFFERSIZE);
    }
}
