#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with components of the ReFlex SF hand
#

from dynamixel_msgs.msg import JointState
import rospy
from std_msgs.msg import Float64


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.name = name[1:]
        self.angle_range = rospy.get_param(self.name + '/angle_range')
        self.flipped = rospy.get_param(self.name + '/flipped')
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.current_raw_position = 0.0
        self.current_pos = 0.0
        self.sub = rospy.Subscriber(name + '/state', JointState,
                                    self.receiveStateCb)

    def receiveStateCb(self, data):
        self.current_raw_position = data.current_pos
        if self.flipped:
            self.current_position = self.zero_point - self.current_raw_position
        else:
            self.current_position = self.current_raw_position - self.zero_point

    def getCurrentPosition(self):
        return self.current_position

    def getRawCurrentPosition(self):
        return self.current_raw_position

    def setMotorZeroPoint(self):
        self.zero_point = self.current_raw_position
        rospy.set_param(self.name + '/zero_point', self.current_raw_position)

    def setMotorPosition(self, goal_pos):
        '''
        Bounds the given motor command and sets it to the motor
        '''
        self.pub.publish(self.checkMotorCommand(goal_pos))

    def setRawMotorPosition(self, goal_pos):
        '''
        Sets the given position to the motor
        '''
        self.pub.publish(goal_pos)

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.flipped:
            tighten_angle *= -1
        self.setRawMotorPosition(self.current_raw_position + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.flipped:
            loosen_angle *= -1
        self.setRawMotorPosition(self.current_raw_position - loosen_angle)

    def checkMotorCommand(self, angle_command):
        '''
        Returns the given command if it's within the allowable range, returns
        the bounded command if it's out of range
        '''
        angle_command = self.correctMotorOffset(angle_command)
        if self.flipped:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self.angle_range)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self.angle_range)
        return bounded_command

    def correctMotorOffset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.flipped:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command
