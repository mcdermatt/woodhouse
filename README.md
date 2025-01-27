# Woodhouse

A tracked robotic platform for LiDAR mapping and valet applications

![WIP](https://img.shields.io/badge/status-WIP-yellow)



<table>
  <tr>
    <td style="text-align: center;">
      <img src="./demo/frontV2.jpg" width="410" />
    </td>
    <td style="text-align: center;">
      <img src="./demo/road.jpg" width="360" />
    </td>
  </tr>
</table>


#### Mapping Overview

<img src="./demo/woodhouse_rqt_graph.png" alt="Node Graph Overview" width="800"/>

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

* gtsam

# Running

#### Controller input

Run node for joystick input

```rosrun joy joy_node```

Run node to convert button output to twist message

```rosparam load $(rospack find woodhouse)/config/joy_config.yaml /teleop_twist_joy```

```rosrun teleop_twist_joy teleop_node```

``` python3 cmd_vel_to_odrive.py```

# TODO

#### Platform

* [ ] publish battery status

#### Mapping and Localization

* [ ] Fix reflections off back of laptop! 
* [X] run ICET odometry
* [X] Explore message passing for scan context features--> Custom keyframe msg type, service, or local storage
* [ ] Detect loop closure canidates
* [ ] topic vs service for fetching clouds from idx for loop closer node
* [ ] Save keyframe point clouds and associated odometry constraints to exteral file
* [ ] compile gtsam with python binds
* [ ] make jupyter notebook to debug odometry constraints output by scan context node
* [ ] Create loop closer node to run ICET again to obtain loop closure constraints


#### Hardware

* Get 8bitdo controller talking with follwing commands:
 
 ```sudo nano /etc/udev/rules.d/99-8bitdo-xinput.rules ```

   ```ACTION=="add", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", RUN+="/sbin/modprobe xpad", RUN+="/bin/sh -c 'echo 2dc8 3106 > /sys/bus/usb/drivers/xpad/new_id'"```