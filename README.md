# flea3

![image](http://www.geeky-gadgets.com/wp-content/uploads/2012/07/Flea3.jpg)

ROS driver for PointGrey Flea3 camera.

See the dynamic reconfigure file for all reconfigurable parameters.

Usage:
```
roslaunch flea3 single_node.launch device:=13344889
```

Dependency:
`camera_base`

[Technical Manual](http://www.ptgrey.com/support/downloads/10120)

[Register Reference](http://www.ptgrey.com/support/downloads/10130/)

## Known Issues

## Optimizing USB performance

http://www.matrix-vision.com/manuals/mvBlueFOX3/mvBC_page_quickstart.html#mvBC_subsubsection_quickstart_linux_requirements_optimising_usb

In `/etc/default/grub`, change
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash" 
```
to
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=256" 
```
then do 
```
sudo update-grub
```
then restart and verify that
```
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```
is `256`

Also it is recommended to upgrade your kernel to 3.16 by doing
```
sudo apt-get install linux-signed-generic-lts-utopic
```
