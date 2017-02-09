#!/bin/bash

echo "DELETE the device serial port(ttyUSBX) to  ganglia"
sudo rm   /etc/udev/rules.d/ganglia.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "