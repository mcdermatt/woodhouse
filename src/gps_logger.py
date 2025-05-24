#!/usr/bin/env python

import rospy
import csv
import os
from sensor_msgs.msg import NavSatFix

class GPSLogger:
    def __init__(self):
        self.filename = os.path.expanduser("~/gps_log.csv")
        self.file = open(self.filename, 'w')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['Latitude', 'Longitude', 'Altitude'])  # CSV header

        rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.callback)
        rospy.loginfo("GPSLogger initialized. Saving data to %s", self.filename)

    def callback(self, msg):
        if msg.status.status >= 0:  # Only log if fix is valid
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            rospy.loginfo("GPS Fix: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt)
            self.csv_writer.writerow([lat, lon, alt])
            self.file.flush()

    def __del__(self):
        self.file.close()

if __name__ == '__main__':
    rospy.init_node('gps_logger', anonymous=True)
    logger = GPSLogger()
    rospy.spin()
