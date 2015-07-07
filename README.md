# ros_verilook

ROS bridge to some of VeriLook SDK's functionality.

### Build
```bash
$ src/ros_verilook/get_sdk.sh
$ catkin_make
```

### Run

```bash
## Outside Docker
# Start the VeriLook license server (binds to the 5000 port).
$ src/ros_verilook/sdk/Bin/Linux_x86_64/Activation/run_pgd.sh start

## Inside Docker
# Tell ROS nodes where to find the license server.
$ rosparam set vl_license_server 172.17.42.1
# 'rosrun' nodes.
$ rosrun usb_cam usb_cam_node _pixel_format:=yuyv
$ rosrun ros_verilook EnrollFaceFromROSTopicNode
$ rosrun ros_verilook identify_face_node.py
```

Skip the `rosparam vl_license_server 172.17.42.1` line if ROS is not running in
docker.

### Test

```bash
$ nosetests-3.4 -v test/unit
```

## identify_face_node.py

Memorizes and identifies faces from a ROS topic image stream.

#### Provides Services
* `/identify_face` - Try to identify the person in the image stream.

For details see: [Identify.srv](srv/Identify.srv), [Match.msg](msg/Match.msg)

* `/save_face` - Memorize a captured face for future identification.

For details see: [Save.srv](srv/Save.srv)

#### Uses Service
* `/create_template` provided by `EnrollFaceFromROSTopicNode`

## EnrollFaceFromROSTopicNode

Written following the `sdk/Tutorials/Biometrics/C/EnrollFace*` type of tutorials.

#### Provides Service
* `/create_template` - in-package use only. Create a VeriLook face template file
on each call. This template can be later used by tutorial programs `Identify`,
`GeneralizeFace` and similar.

#### Consumes Topic
* `/usb_cam/image_raw` - camera image stream.
