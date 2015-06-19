# ros_verilook

ROS bridge to some of VeriLook SDK's functionality.

## EnrollFaceFromROSTopic Node

Written following the `sdk/Tutorials/Biometrics/C/EnrollFace` type of tutorials.

**Provides Service** `/create_template` -  creates a VeriLook face template file on each call. This template can be later used by tutorial programs `Identify`, `GeneralizeFace` and similar.

**Consumes topic** `/usb_cam/image_raw`

### Build
```bash
$ src/ros_verilook/get_sdk.sh
$ catkin_make
```

### Run
```bash
$ rosrun ros_verilook EnrollFaceFromROSTopic
```
