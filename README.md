# Automated Pick-and-Place of Balls using a Franka Emika Robot and ZED 2i Camera
Github Link to this repository: (https://github.com/nkenny160/ecen524-final-team6/tree/main)

This repository contains the code we used to implement automated pick and place using the Franka Emika Panda Robot and a Stereolabs ZED 2i camera. The code is well documented and fairly easy to understand. In order to run this code, please follow the steps below.

## Preface
Unfortunately, due to the scope of this project, we do not believe it is reasonable for someone to install dependencies and set up our workspace on a different machine than the one the project was done on. It took us the better part of a week to get everthing set up with the Franka Emika robot, ROS2, Franka ROS, and many more things. However these steps will definitely get you close, but debugging skills and ROS experience is required.

## Steps to Replicate our Results
### Step 1 - Environment setup
Install ROS2 Humble with RViz on either an Nvidia Jetson with Jetpack 6.2 (and preferrably using an SSD not SD card) or x86 desktop with Ubuntu 22.04. The minimum desktop hardware we would recommend is an RTX-3060 (we used an RTX-3070), 6-core CPU, and 32GB of ram; it is possible to get by with less ram but this will make installation take longer. A graphics processor capable of supporting Cuda 12.8 and TensorRT 10 is required. Additionally, you will require a ZED stereo camera and the appropriate SDK installed (https://www.stereolabs.com/docs/installation/linux). We used ZED SDK 5.0.0 EA and a ZED 2i with the latest firmware.

### Step 2 - Create ROS2 Workspace
Create a new ROS workspace:
```
mkdir workspace
cd workspace
mkdir workspace\src
cd src
```
#### In this new workspace Git clone the following repositories using their installation instructions minus colcon build steps:
- [Multi-Franka-Arm-ROS2](https://github.com/yilmazabdurrah/multi_franka_arm_ros2)
- [MoveIt 2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
- [ZED ROS Wrapper](https://github.com/stereolabs/zed-ros-wrapper/tree/master)
- [ZED ROS Examples](https://github.com/stereolabs/zed-ros-examples)

### Step 3 - Prepare to build
Next git clone our repository into the same source folder. Once it is cloned, replace the folders ```franka_description``` and ```franka_moveit_config``` in the ```multi-franka-arm-ros2``` folder with the folders from our repository. Leave auto_pick_place_moveit in the ```workspace\src```.

Then update ```auto_pick_place_moveit.cpp``` with your new transformation matrix. This transformation should be approximately from the base link of the Franka Emika Panda to the left camera frame on a ZED stereo camera. We did this with simply a measuring tape and it seemed to work reasonbly well, but other methods - such as using calibration markers - are also possible. This matrix defines the transformation used to calculate goal poses for the Panda robot based on the detected object positions relative to the ZED camera running object detection. Update this code with your transformation matrix:
```
    Eigen::Matrix4f T1_0;
    T1_0 << 0.0, 1.0, 0.0, 0.2016,
        -0.9511, 0.0, -0.3090, 1.0604,
        -0.3090, 0.0, 0.9511, 0.3429,
        0.0, 0.0, 0.0, 1.0;
```

Next update ```stereo_common.yaml``` configuration file in the ZED ROS wrapper folder. We need to change the object detection section to enable object detection by setting the option to ```true``` and only enable the classes which you want to recognize. For our project, we only set "sports" to true so that we could only recognize balls.

### Step 4 - Build
Make sure that you have initialized ```rosdep``` updated it and and installed any dependencies for other packages along the way. Then move to ```\workspace``` (above src) and you can build using the command:
```
colcon build --mixin release
```
You may need to add the flags ```MAKEFLAGS="-j4 -l1"``` before "colcon" and ```--executor sequential``` before "release" if you are working with a limited system.

Make sure to source your workspace in the bashrc file.

### Step 5 - Running
First launch the ZED object detection visualizer using 
```
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```
Make sure to change the camera model if you are not using a ZED 2i. Object detections can be viewed visually in RViz or textually in another terminal using 

```
ros2 topic echo /zed/zed_node/obj_det/objects
```
This ROS topic is also the topic that our package subscribes to in order to generate positon goals by itself.

Then enable FCI on the Franka Emika Panda and in another terminal run 
```
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<insert_ip_here> use_fake_hardware:=false
```
This starts RViz with a visual representation of the Panda robot and MoveIt controller for all joints and the end-effector. You can run this package in simulation instead (without a real robot) using
```
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```
Then, in a third terminal, launch our package to begin moving the balls using the command 
'''
ros2 run auto_pick_place_moveit auto_pick_place_moveit
'''
After running this launch command, the robot will first calibrate the gripper and then iteratively start moving to grab balls to be placed in the desired location. In between movements, the Panda arm will go back to it's home position to avoid collisions and singularities. 

### Step 6 - Iterative Improvements
Most of the changes you need to make will probably be to the logic in ```auto_pick_place_moveit.cpp```. To easily rebuild just this package use:

```
colcon build --packages-select auto_pick_place_moveit
```

To choose a different place to drop the ball, manually move the arm to your desired position and then briefly run the command

```
ros2 topic echo /joint_states
```
and use these values to update the following joint position vector in ```auto_pick_place_moveit.cpp``` (vector is in normal order from joint0 to joint7):
```
std::vector<double> target_joint_positions = {0.0, -0.785, 0, -2.356, 0, 1.571, 0.785};
```
Remember that any changes to this C++ file will require you to colcon build the package again, see above for a tip.

## Enjoy! Expected results, or at least our results
Watch this [YouTube video](https://youtu.be/ft46-UO4ZqM?si=W7TRYww2xCleK4V9) to see the whole thing in action.