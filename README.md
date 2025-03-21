# ecen524-final-team6
## Installation Instructions
Unfortunately, due to the scope of this project, we do not believe it is reasonable for someone to install dependencies and set up our workspace on a different machine than the one the project was done on. It took us the better part of a week to get everthing set up with the Franka Emika robot, ROS2, Franka ROS, and many more things. Instead, we will breifly describe the installation process that we followed.

- Install [ROS2 (Humble)](https://docs.ros.org/en/humble/Installation.html).
- Install [ZED](link to zed tutorial) (ADD DETAILS HERE).
- Install [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html).
- Create a Colcon workspace and set up the [multi_franka_arm_ros2](https://github.com/yilmazabdurrah/multi_franka_arm_ros2) port. This port allows Franka ROS2 to be used with the older (FER) Franka Emika Panda robot models.
- In the same workspace, install and build [MoveIt 2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html).
- In the same workspace, create a new package called "auto_pick_place_moveit" containing the files in the folder with the same name in this code submission. [This tutorial](https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html) will help with this process.
- Build you workspace.
- WE SHOULD PROBABLY ADD SOME NOTES HERE ABOUT ANY CUSTOM STUFF WE HAD TO DO

## Usage Instructions

## Expected Output
