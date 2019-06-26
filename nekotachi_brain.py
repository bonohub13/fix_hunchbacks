#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from scipy import arctan, pi

from sensor_msgs.msg import Imu

from k_means import k_means, data_6d, data_9d

INPUT_DATA = 'imu/data_raw'

class Nekotachi_think:
    def __init__(self):
        self.imu_msgs = Imu()
        rospy.Subscriber(INPUT_DATA, Imu, self.imu_callback)
        self.__angle = self.angle_data
        self.linear, self.angular_v, self.angle = self.collect_data()

    def imu_callback(self, imu):
        self.imu_msgs = imu

    def collect_data(self):
        linear = []

        linear.append(self.imu_msgs.linear_acceleration.x)
        linear.append(self.imu_msgs.linear_acceleration.y)
        linear.append(self.imu_msgs.linear_acceleration.z)
        
        angular = []

        angular.append(self.imu_msgs.angular_velocity.x)
        angular.append(self.imu_msgs.angular_velocity.y)
        angular.append(self.imu_msgs.angular_velocity.z)

        angle = self.__angle(linear, angular)

        return linear, angular, angle

    def angle_data(self, linear, angular):
        angle_C = []

        for i in range(len(linear)):
            if (linear[i] < 0.01) and (-0.01 < linear[i]):
                linear[i] = 0
        if linear[2] != 0:
            angle_C.append(arctan(linear[1]/linear[2])/pi*180)
        else:
            angle_C.append(arctan(linear[1]/0.01)/pi*180)
        if linear[2] != 0:
            angle_C.append(arctan(linear[0]/linear[2])/pi*180)
        else:
            angle_C.append(arctan(linear[0]/0.01)/pi*180)
        if linear[0] != 0:
            angle_C.append(arctan(linear[1]/linear[0])/pi*180)
        else:
            angle_C.append(arctan(linear[1]/0.01)/pi*180)

        angle = []

        for j in range(3):
            angle.append(angular[j]*0.01 + angle_C[j])

        return angle

    def return_data(self):
        self.linear, self.angular_v, self.angle = self.collect_data()

    def run(self):
        self.linear, self.angular_v, self.angle = self.collect_data()
        print('============================================\n')
        print('---------- angular data --------------------')
        print('angular_velocity   : %f %f %f' % (self.angular_v[0], self.angular_v[1], self.angular_v[2]))
        print('angle              : %f %f %f' % (self.angle[0], self.angle[1], self.angle[2]))
        print('---------- linear data ---------------------')
        print('linear acceleration: %f %f %f' % (self.linear[0], self.linear[1], self.linear[2]))
        print('\n============================================\n')

class K_means(Nekotachi_think):
    def __init__(self):
        super().__init__()
        self.data_6d = []
        self.data_9d = []
        self.k_means6 = k_means(data_6d)
        self.k_means9 = k_means(data_9d)

    def data_out(self):
        print(self.data_6d)

    def k_6neko(self):
        keys_k_meaned = list(self.k_means6.keys())
        keys_6 = list(self.k_means6[keys_k_meaned[0]].keys())[:]
        dist = []
        for j in range(len(self.k_means6)):
            dist.append(sum([(self.k_means6[keys_k_meaned[j]][key] - self.data_6d[k])**2\
                for k, key in zip(range(len(keys_6)), keys_6)]))
        if dist[0] == min(dist):
            print('You\'re === A ===')
        elif dist[1] == min(dist):
            print('You\'re === B ===')
        elif dist[2] == min(dist):
            print('You\'re === C ===')
        elif dist[3] == min(dist):
            print('You\'re === D ===')
        else:
            print('You\'re === F ===')

    def k_9neko(self):
        keys_k_meaned = list(self.k_means9.keys())
        keys_9 = list(self.k_means9[keys_k_meaned[0]].keys())[:]
        dist = []
        for j in range(len(self.k_means9)):
            dist.append(sum([(self.k_means9[keys_k_meaned[j]][key] - self.data_9d[k])**2\
                for k, key in zip(range(9), keys_9)]))
        if dist[0] == min(dist):
            print('You\'re === A ===')
        elif dist[1] == min(dist):
            print('You\'re === B ===')
        else:
            print('You\'re === C ===')

    def __data_6d_out(self):
        tmp_6d = []
        linear, angular_v, angle = self.collect_data()
        for lin in linear:
            tmp_6d.append(lin)
        for ang in angle:
            tmp_6d.append(ang)
        return tmp_6d

    def __data_9d_out(self):
        tmp_9d = []
        linear, angular_v, angle = self.collect_data()
        for lin in linear:
            tmp_9d.append(lin)
        for ang in angle:
            tmp_9d.append(ang)
        for ang_v in angular_v:
            tmp_9d.append(ang_v)
        return tmp_9d

    def main(self):
        counter = 0
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if counter > 4:
                self.data_6d = self.__data_6d_out()[:]
                self.data_9d = self.__data_9d_out()[:]
                self.k_6neko()
            else:
                counter += 1
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('nekotachi_brain')
    brain = K_means()
    brain.main()