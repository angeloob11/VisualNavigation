# Visual Navigation with behavior trees

This repository is the main part of my degree work containing 4 packages for the simulation of the navigation of a mobile robot using behavior trees and computer vision. All was build using ROS2-Humble ang Gazebo-Ignition.

to use the packages in this repository:

 - create a workspace folder and a src folder inside it.
 - `git clone https://github.com/angeloob11/VisualNavigation.git` in the src folder
 - `colcon build --symlink-install`

The package **world_gen** is a modification of the following repository: https://github.com/azazdeaz/fields-ignition.git, this creates the world, to use it run the jupyter notebooks in the scripts folder, one of them creates the world without the husky model used in one of the simulations. remember built this package every time that a new world is created.

The package **segmentation_camera** runs a simulation for obtaining segmentated images for training, to use it launch the file **launch_ign.launch.py**, make sure that the path to the world inside it is corect. to control the robot movement use the next command:

 - `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/my_robot/cmd_vel`

Finally, the packages **img_treat** and **nav_control** are the main packages for controlling the visual navigation simulation. to use them simply launch **launch_world.launch.py** and, once the world is loaded, launch **launch_nav.launch.py** to use the navigation, these files belong to the **nav_control** package. For monitoring the Behavior Tree run groot an use the 2666 and 2667 ports.

If you want to contact me, you can write at **angeloob11@gmail.com**
