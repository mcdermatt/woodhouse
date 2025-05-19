# Woodhouse

A tracked robotic platform for LiDAR mapping

![WIP](https://img.shields.io/badge/status-WIP-yellow)


<table>
  <tr>
    <td style="text-align: center;">
      <img src="./demo/woodhouseCAD.jpg" width="410" />
    </td>
    <td style="text-align: center;">
      <img src="./demo/stairs.gif" width="360" />
    </td>
  </tr>
</table>

#### Mapping Overview

<img src="./demo/woodhouse_rqt_graph.png" alt="Node Graph Overview" width="800"/>

* ```rosrun icet odometry_node``` or ```rosrun icet map_maker_node```
    * Produces raw LiDAR odometry estimates from sensor data stream
* ```rosrun woodhouse scan_context_node```
    * Generates new keyframe if the platform has moved enough since the last keyframe
    * If there's enough similarity between nearby keyframes, asks loop closer node (via pose graph node) to register the two clouds in order to add an additional constraint to the graph.
* ```rosrun woodhouse loop_closer_node``` or ```rosrun woodhouse loop_closer_eigen_node```
    * Runs scan registration between arbitrary keyframes
* ```rosrun woodhouse pose_graph_node```
    * holds on to all keyframe point clouds and assicoated absolute positions. Uses iSAM2 to optimize all constraints.  

<img src="./demo/indoorHDMap.jpg" alt="Indoor HD Map" width="800"/>

Example map generated in real time by driving the above system around my apartment at high speed. Red dots represent optimized keyframe locations.

# Requirements 
* joy

```sudo apt-get install ros-noetic-joy```

* icet 

* teleop-twist-joy
```
cd ~/catkin_ws/src
git clone https://github.com/mcdermatt/icet
git clone https://github.com/ros-teleop/teleop_twist_joy
catkin_make
```

* install 8bitdo controller ROS driver:
 
 ```
 sudo nano /etc/udev/rules.d/99-8bitdo-xinput.rules 

 ACTION=="add", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", RUN+="/sbin/modprobe xpad", RUN+="/bin/sh -c 'echo 2dc8 3106 > /sys/bus/usb/drivers/xpad/new_id'"
 ```

* gtsam
```
cd ~/catkin_ws/src
git clone https://github.com/borglab/gtsam.git
git checkout 4.3.0 

mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_PACKAGING=ON
make -j$(nproc)
sudo make install

```
https://github.com/MIT-SPARK/Kimera-VIO-ROS/issues/12

https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/247 

use the following command to make sure we are linking our executables against the correct installation of GTSAM 

```
ldd ~/ROS/devel/lib/woodhouse/pose_graph_node | grep gtsam

```

* install Velodyne drivers
```
git clone https://github.com/ros-drivers/velodyne/tree/dashing-devel
``` 


# Running

### Velodyne LiDAR Sensor

```
roslaunch velodyne_pointcloud VLP-32C_points.launch
```

### Controller input

Run node for joystick input

```rosrun joy joy_node```

Run node to convert button output to twist message

```rosparam load $(rospack find woodhouse)/config/joy_config.yaml /teleop_twist_joy```

```rosrun teleop_twist_joy teleop_node```

``` python3 cmd_vel_to_odrive.py```

# TODO

#### Mapping and Localization

* [ ] Use ICP convergence stability as test for solution quality
* [ ] Move to more efficient sliding window for graph pruning

#### Hardware

* [ ] Push CAD 4.1

#### Platform

* [ ] publish battery status
* [ ] consolidate everything into single launch file
