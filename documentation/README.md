Installation Peak CAN bus - USB converter
```bash
tar -xzf peak-linux-driver-8.5.1.tar.gz 
cd ~/PEAK/peak-linux-driver-8.5.1
make clean
make -C driver NET=NETDEV_SUPPORT
make -C lib
make -C test
sudo make install
sudo modprobe pcan
cat /proc/pcan
```

If it does not work, please run:
```bash
sudo rmmod pcan
sudo modprobe pcan
cat /proc/pcan
```

Activation and desactivation of peak CAN bus
```bash
sudo ifconfig can0 up
sudo ifconfig can0 down
```

Installation of can-utils
```bash
sudo apt-get install can-utils
candump can0 // Watch the raw data received once the Peak CAN bus is installed and connected to Radar
```
