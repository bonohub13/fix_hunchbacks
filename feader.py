#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import numpy as np
from scipy import arctan, pi

from sensor_msgs.msg import Imu

DATA_RAW = "imu/data_raw"
DATA_WITH_ANGULAR = "~/workspace/neko_ws/src/ros_neko/src/data/angular_data.txt"
DATA_ONLY_ANGLE = "~/workspace/neko_ws/src/ros_neko/src/data/position_data.txt"

class Data:
    def __init__(self):
        self.imu_msgs = Imu()
        rospy.Subscriber(DATA_RAW, Imu, self.imu_callback)
        self.angle_data = self.angle

    def imu_callback(self, imu):
        self.imu_msgs = imu

    def get_data(self):
        lin = (\
            self.imu_msgs.linear_acceleration.x,\
            self.imu_msgs.linear_acceleration.y,\
            self.imu_msgs.linear_acceleration.z)
        angular = (\
            self.imu_msgs.angular_velocity.x,\
            self.imu_msgs.angular_velocity.y,\
            self.imu_msgs.angular_velocity.z)
        angle = self.angle_data(list(lin), list(angular))
        print(lin, angular, angle)

        return lin, angular, angle
    
    def angle(self, lin, angular):
        angle_C = []
        for i in range(len(lin)):
            if (lin[i] < 0.01) and (-0.01 < lin[i]):
                lin[i] = 0
        if lin[2] != 0:
            angle_C.append(arctan(lin[1]/lin[2])/pi*180)
        else:
            angle_C.append(arctan(lin[1]/0.01)/pi*180)
        if lin[2] != 0:
            angle_C.append(arctan(lin[0]/lin[2])/pi*180)
        else:
            angle_C.append(arctan(lin[0]/0.01)/pi*180)
        if lin[0] != 0:
            angle_C.append(arctan(lin[1]/lin[0])/pi*180)
        else:
            angle_C.append(arctan(lin[1]/0.01)/pi*180)
        return tuple(angle_C)

    def run(self):
        lin, angular, angle = self.get_data()
        os.system('echo \'%f %f %f %f %f %f %f %f %f\' >> %s' % \
            (lin[0], lin[1], lin[2], angular[0], angular[1], angular[2], angle[0], angle[1], angle[2], DATA_WITH_ANGULAR))
        os.system('echo \'%f %f %f %f %f %f\' >> %s' % \
            (lin[0], lin[1], lin[2], angle[0], angle[1], angle[2], DATA_ONLY_ANGLE))

    def main(self):
        os.system('rm %s %s' % (DATA_ONLY_ANGLE, DATA_WITH_ANGULAR))
        count = 0
        rate_n = 100
        rate = rospy.Rate(rate_n)
        while not rospy.is_shutdown():
            if count > 4:
                self.run()
            count += 1
            if count == 120*rate_n+5:
                exit()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('nekotachi')
    Data().main()
