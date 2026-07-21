#!/bin/sh
oldpwd=$(pwd)
cd $(dirname $0)
sudo cp 60-openocd.rules /etc/udev/rules.d/
sudo cp 66-wiced-JTAG.rules /etc/udev/rules.d/
sudo service udev restart
sudo udevadm control --reload
sudo udevadm trigger --action=add
cd $oldpwd
