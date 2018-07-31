#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys


class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        filename = 'IMG_TIMESTAMP.txt'
        with rosbag.Bag('/home/haosu/Desktop/Ithinkok/walkout_30_200.bag', 'r') as bag:
            for topic,msg,t in bag.read_messages():
                if topic == "/downward/image_raw":
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")
                        except CvBridgeError as e:
                            print "error"
                        timestr = "%d" %  int(msg.header.stamp.to_sec()*1e9)
                        with open(filename,'a') as f:
                            f.write(timestr)
                            f.write('\n')
                        image_name = timestr+ ".png"
                        print image_name
                        cv2.imwrite(image_name, cv_image)
                #if topic == "/imu0":
                #    imu_timestamp = "%.9f" % msg.header.stamp.to_sec()
                #    imu_angular_velocity_x = msg.linear_acceleration.x
                #    print imu_angular_velocity_x
                #    print imu_timestamp

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
