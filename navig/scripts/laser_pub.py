#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def laser_scan_cb(data):
    print("data")
    pub = rospy.Publisher(data.header.frame_id.split('/')[1]+"/scan", LaserScan, queue_size=100)
    rate = rospy.Rate(10)

if __name__ == "__main__":
    rospy.init_node("laser_scan_node", anonymous=True)
    try:
	for i  in range(1):
        	sub = rospy.Subscriber("/X1/front_scan", LaserScan, laser_scan_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
