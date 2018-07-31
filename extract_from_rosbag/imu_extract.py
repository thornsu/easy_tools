#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy

#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys


class IMUCreator():


    def __init__(self):
        #self.bridge = CvBridge()
        filename = 'IMU_data.csv'
        with open(filename,'w') as f:
            f.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')
        with rosbag.Bag('/home/haosu/Desktop/Ithinkok/walkout_30_200.bag', 'r') as bag:
            for topic,msg,t in bag.read_messages():
                if topic == "/imu0":
                    imu_timestamp = "%d" %  int(msg.header.stamp.to_sec()*1e9)
                    imu_omega_x = "%.16f" %msg.angular_velocity.x
                    imu_omega_y = "%.16f" %msg.angular_velocity.y
                    imu_omega_z = "%.16f" %msg.angular_velocity.z
                    imu_alpha_x = "%.16f" %msg.linear_acceleration.x
                    imu_alpha_y = "%.16f" %msg.linear_acceleration.y
                    imu_alpha_z = "%.16f" %msg.linear_acceleration.z
                    with open(filename,'a') as f:
                        f.write(imu_timestamp)
                        f.write(',')
                        f.write(imu_omega_x)
                        f.write(',')
                        f.write(imu_omega_y)
                        f.write(',')
                        f.write(imu_omega_z)
                        f.write(',')
                        f.write(imu_alpha_x)
                        f.write(',')
                        f.write(imu_alpha_y)
                        f.write(',')
                        f.write(imu_alpha_z)
                        f.write('\n')


                #if topic == "/imu0":
                #    imu_timestamp = "%.9f" % msg.header.stamp.to_sec()
                #    imu_angular_velocity_x = msg.linear_acceleration.x
                #    print imu_angular_velocity_x
                #    print imu_timestamp

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        imu_creator = IMUCreator()
    except rospy.ROSInterruptException:
        pass
