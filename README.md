# ros_verilook

ROS bridge to some of VeriLook SDK's functionality.

### Build
```bash
$ src/ros_verilook/get_sdk.sh
$ catkin_make
```

### Run
```bash
$ rosrun ros_verilook EnrollFaceFromROSTopic
```

## identify_face_node.py

*..under development..*

Memorizes and identifies faces from a ROS topic image stream.

#### Provides Services
* `/identify_face` - Try to identify the person in the image stream. Returns a *handle* to this face and its position in picture. Responds with an error if unable to find a face.

* `/save_face` - Memorize a face for future identification. Takes a *handle* to the face provided by the `/identify_face` service.

#### Uses Service
* `/create_template` provided by `EnrollFaceFromROSTopicNode`

## EnrollFaceFromROSTopicNode

Written following the `sdk/Tutorials/Biometrics/C/EnrollFace*` type of tutorials.

#### Provides Service
* `/create_template` -  Create a VeriLook face template file on each call. This template can be later used by tutorial programs `Identify`, `GeneralizeFace` and similar.

#### Consumes Topic
* `/usb_cam/image_raw`
