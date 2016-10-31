echo 'SUBSYSTEMS=="usb", ATTR{idVendor}=="03eb", SYSFS{idProduct}=="2ff4", MODE="660", GROUP="dialout" ' | sudo tee /etc/udev/rules.d/atmel.rules
sudo udevadm trigger
sudo udevadm control --reload-rules
