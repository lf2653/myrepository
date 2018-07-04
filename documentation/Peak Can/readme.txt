# Installation Peak CAN bus - USB converter

-> cd ~/PEAK/peak-linux-driver-8.5.1
-> make clean
-> make -C driver NET=NETDEV_SUPPORT
-> sudo make install
-> sudo modprobe pcan
-> cat /proc/pcan

# Activation and desactivation

-> sudo ifconfig can0 up
-> sudo ifconfig can0 down

# If it does not work, please run:

-> sudo rmmod pcan
-> sudo modprobe pcan
-> cat /proc/pcan

# Installation of can-utils

-> sudo apt-get install can-utils
