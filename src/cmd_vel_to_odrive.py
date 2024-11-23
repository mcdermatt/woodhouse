#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import odrive
from odrive.enums import*

class cmd2odrv():

    def __init__(self):

        #init ODrive
        serial0 = "206A337B304B"
        self.odrv0 = odrive.find_any(serial_number=serial0)
        print("found od0")
        # odrv0.axis0.controller.config.control_mode = 2
        # odrv0.axis1.controller.config.control_mode = 2

        # Initialize the ROS node
        rospy.init_node('cmd_vel_listener', anonymous=True)
        # Subscribe to the 'cmd_vel' topic, which publishes Twist messages
        rospy.Subscriber("cmd_vel", Twist, self.callback)

    def callback(self, msg):
        # Extract and print the x-component of linear velocity
        rospy.loginfo("Linear x velocity: %f, Angular z velocity: %f", msg.linear.x, msg.angular.z)    

        # scale = 1000 #too fast
        scale = 200

        L_wheel_setpoint = scale*(msg.linear.x + msg.angular.z)
        R_wheel_setpoint = -scale*(msg.linear.x - msg.angular.z)

        self.odrv0.axis0.controller.vel_setpoint = L_wheel_setpoint
        self.odrv0.axis1.controller.vel_setpoint = R_wheel_setpoint

if __name__ == '__main__':
    cmd_vel_to_odriv = cmd2odrv()

    while not rospy.is_shutdown():
        rospy.spin()