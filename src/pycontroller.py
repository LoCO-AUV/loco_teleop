#!/usr/bin/env python

# This code is a part of the LoCO AUV project.
# Copyright (C) The Regents of the University of Minnesota

# Maintainer: Junaed Sattar <junaed@umn.edu> and the Interactive Robotics and Vision Laboratory

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rospy
from sensor_msgs.msg import Joy
from loco_pilot.msg import Command
import mavros
import mavros.command

cmdPub = None

def joy_callback(data):
    global cmdPub
    (yaw, throttle, _, pitch, _, _) = data.axes
    msg = Command()

    msg.pitch = pitch
    msg.yaw = yaw*-1
    msg.throttle = throttle

    cmdPub.publish(msg)

def spinner():
    global cmdPub
    rospy.init_node('loco_teleop', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber("/joy", Joy, joy_callback)
    cmdPub = rospy.Publisher("/loco/command", Command, queue_size=5)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        spinner()
    except rospy.ROSInterruptException:
        pass
