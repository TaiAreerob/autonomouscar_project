#!/bin/bash 

sudo touch /etc/udev/rules.d/10-local.rules

echo ' ACTION=="add", ATTRS{serial}=="FTFJKS5O", OWNER="ssora", GROUP="ssora", SYMLINK+="erp42" ' >> /etc/udev/rules.d/10-local.rules

echo ' ACTION=="add", ATTRS{serial}=="A105NFWS", OWNER="ssora", GROUP="ssora", SYMLINK+="erp42" ' >> /etc/udev/rules.d/10-local.rules 

echo ' ACTION=="add", ATTRS{serial}=="AH03H2SL", GROUP="ssora", OWNER="ssora", SYMLINK+="garmin19x" ' >> /etc/udev/rules.d/10-local.rules 

echo ' ACTION=="add", ATTRS{serial}=="0001", GROUP="ssora", OWNER="ssora", SYMLINK+="mysen_imu" ' >> /etc/udev/rules.d/10-local.rules

echo "don't exec again"
echo "check /etc/udev/rules.d/" 

