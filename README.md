# flea3

![image](http://www.ptgrey.com/content/images/thumbs/0002327_flea3-88-mp-color-usb3-vision-sony-imx121_772.jpeg)

Another ROS driver for Point Grey Flea3 camera.

**Note**:

It is called `flea3` instead of `pointgrey_camera_driver` because I only have flea3 cameras. So if you have another point grey camera, it might not work.

[Flea3](http://www.ptgrey.com/flea3-usb3-vision-cameras)

Dependency:
[`camera_base`](https://github.com/KumarRobotics/camera_base)

## Supported hardware

This driver should work with any Point Grey Flea3 usb3.0 cameras. I only tested it with [FL3-U3-13E4C-C](http://www.ptgrey.com/flea3-13-mp-color-usb3-vision-e2v-ev76c560-camera)

See the dynamic reconfigure file for all reconfigurable parameters.

## API Stability
The ROS API of this driver should be considered unstable.

## ROS API
`single_node` is a node for a single flea3 camera.

#### Published topics

`~image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

The unprocessed image data.

`~camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

Contains the camera calibration (if calibrated) and extra data about the camera configuration.

#### Parameters

See the dynamic reconfigure file for all reconfigurable parameters.

Usage:
* single camera
```
roslaunch flea3 single_node.launch device:=13344889
```
* stereo camera
```
roslaunch flea3 stereo_node.launch left:=13344889 right:=14472242
```

## FlyCapture2

FlyCapture2 can be downloaded from [here](http://www.ptgrey.com/support/downloads)

[Technical Manual](http://www.ptgrey.com/support/downloads/10120)

[Register Reference](http://www.ptgrey.com/support/downloads/10130/)

## Known Issues

## Optimizing USB performance

Here is an [article](http://www.matrix-vision.com/manuals/mvBlueFOX3/mvBC_page_quickstart.html#mvBC_subsubsection_quickstart_linux_requirements_optimising_usb) from matrix-vision on how to optimize USB performance.

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
