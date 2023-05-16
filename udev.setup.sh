sudo rm /etc/udev/rules.d/99-vesc.rules
sudo sh -c 'echo KERNEL==\"ttyACM[0-9]*\", ACTION==\"add\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", SYMLINK+=\"sensors/vesc\" >> /etc/udev/rules.d/99-vesc.rules'
sudo rm /etc/udev/rules.d/99-ps4.rules
sudo sh -c 'echo KERNEL==\"js[0-9]*\", ACTION==\"add\", ATTRS{idVendor}==\"05e3\", ATTRS{idProduct}==\"0608\", SYMLINK+=\"sensors/ps4\" >> /etc/udev/rules.d/99-ps4.rules'

