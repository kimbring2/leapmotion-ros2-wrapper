# Introduction
This package lets you use the Leap Motion optical hand tracking module with ROS2. It provides access to the following data:
- Fingers, bones position for the left, and right hand.

This ROS2 package can be used with [leapmotion_display_rviz2](https://github.com/kimbring2/leapmotion-ros2-examples/tree/main/leapmotion_display_rviz2) packages to visualize the hand tracking at RViz2.

<img src="images/leapmotion_rviz.gif" width="1000">

# Known issues
This package is not official and developed by myself alone. Even if I borrow the really good ROS2 format from [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) package, there are still lots of parts those should be fixed. The pull request is always welcome (:

# Requirements
- Ubuntu 20.04
- ROS2 Foxy

# Build the package
```
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone  --recursive https://github.com/kimbring2/leapmotion-ros2-wrapper.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```

**Note**: If rosdep is missing you can install it with:

$ sudo apt-get install python-rosdep python-rosinstall-generator python-vcstool python-rosinstall build-essential

#  Starting the Leapmotion node
```
$ ros2 launch leapmotion_wrapper leapmotion.launch.py
```

After starting node, you should see the below topic list.

```
$ ros2 topic list
```

<img src="images/leapmotion_topic.png" width="1000">
