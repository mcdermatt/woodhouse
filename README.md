# woodhouse

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

```rosrun teleop_twist_joy teleop_node```

# TODO

#### General
* [X] Set up repo

#### Basic motion

* [ ] Convert controller input to twist msg
* [ ] convert twist to odrive command

#### Mapping and Localization

* [ ] run ICET odometry
* [ ] Add simple histogram criteria for loop closure
* [ ] compile gtsam for pose factors
* [ ] get factor graph node working 

#### Hardware

* [ ] Add CAD to repo
* [ ] Order 80/20
* [ ] Spec dampers
* [ ] Get 8bitdo controller working
 
 ```sudo nano /etc/udev/rules.d/99-8bitdo-xinput.rules ```

   ```ACTION=="add", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="3106", RUN+="/sbin/modprobe xpad", RUN+="/bin/sh -c 'echo 2dc8 3106 > /sys/bus/usb/drivers/xpad/new_id'"```
