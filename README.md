vehicle_reaction_ros2
====
This project is part of my master thesis: "Implementation of a System to Infrastructure System for Autonomous Vehicles"

Set up GitHub
------
https://docs.github.com/en/get-started
```
git config --global user.name "PedroSoler10"
git config --global user.email "soler.pedrojavier@gmail.com"
```
SSH authentication:

https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh

https://github.com/settings/keys

```
ssh-keygen -t ed25519 -C "soler.pedrojavier@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/max_laptop
cat ~/.ssh/max_laptop.pub
```

Clone repository:
```
git clone git@github.com:PedroSoler10/camera_infrastructure.git
```

ROS2 Tutorials
----
https://docs.ros.org/en/foxy/Tutorials.html

Create a workspace
----
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
```
colcon build --symlink-install 
```

Create a ROS2 package
----
https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

Create the package:
```
ros2 pkg create --build-type ament_cmake <package_name>
colcon build --packages-select <package_name>
```

Create a node
----
Create a .py file in ws/src/pkg_name/pkg_name, make it executable with chmod +x and add in the setup.py:
```
    entry_points={
        'console_scripts': [
            'reactor_node = reaction_pkg.reactor_node:main',
        ],
    },
```
Run a node
----
```
cd ~/vehicle_ws  # or the root of your ROS2 workspace
source install/setup.bash
ros2 run reaction_pkg reactor_node
```

Use ros1_bridge
----
Install ros1_bridge
```
sudo apt install ros-foxy-ros1-bridge
```
Open a terminal and source the ROS1 environment and run roscore:
```
source /opt/ros/noeticsetup.bash
roscore
```
Open a terminal and source the ROS1 and ROS2 environments and run ros1_bridge:
```
source /opt/ros/noeticsetup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge
```
Run ROS1 nodes in terminals sourced with ROS1 and the same for ROS2 nodes.

reaction_pkg
----
```
source-ros2
rosrun reaction_pkg reactor_node
```

Connecting two devices via WLAN and ROS
----
Server device (where roscore will be running):
```
export ROS_MASTER_URI=http://10.42.0.85:11311
export ROS_HOSTNAME=10.42.0.85
```
Client device:
```
export ROS_MASTER_URI=http://10.42.0.85:11311
export ROS_HOSTNAME=10.42.0.1
```
The IPs shown correspond to the HP (85) and Max (1) laptops when connected to Max's hotspot.


