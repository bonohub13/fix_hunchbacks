#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt16

class IdentifyNeko:
    def __init__(self):
        self.data_msgs = UInt16()
        rospy.Subscriber("judge", UInt16, self.data_callback)
        self.identifier = []
        self.checker = [1 for i in range(300)]

    def data_callback(self, data):
        self.data_msgs = data

    def make_identifier(self):
        self.identifier.append(self.data_msgs.data/100)
        if len(self.identifier) > 350:
            self.identifier.pop(0)

class Shame_inator(IdentifyNeko):
    def __init__(self):
        super().__init__()
        rospy.Subscriber("/buzzer", UInt16, self.buzzer_callback)

    def buzzer_freq(self, hz=0):
        buzzer_file = '/dev/rtbuzzer0'
        try:
            with open(buzzer_file, 'w') as f:
                f.write(str(hz)+'\n')
        except IOError:
            rospy.logerr('Couldn\'t access %s.' % buzzer_file)

    def buzzer_callback(self, buzz):
        self.buzzer_freq(buzz.date, hz=self.data_msgs.data)

    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.make_identifier()
            print(self.identifier)
            self.buzzer_freq()
            if self.identifier[-300:] == self.checker:
                print('===== MATCH! =====')
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("shame_neko")
    shame = Shame_inator()
    shame.main()
