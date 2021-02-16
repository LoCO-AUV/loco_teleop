#!/usr/bin/env python3

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



#This file is based on teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

#from geometry_msgs.msg import Twist
from loco_pilot.msg import Command

import sys, select, termios, tty


msg = """
Reading from the keyboard
---------------------------
Planar Movement:
   q    w    e
   a         d
   z    x    c

i : up (+z)
k : down (-z)

Anything Else : stop

r/v : increase/decrease thruster power by 10% of full
---------------------------
CTRL-C to quit
"""

#pitch-yaw-throttle
moveBindings = {
        'q':( 0,-(2**0.5)/2, (2**0.5)/2, 0),
        'w':( 0, 0, 1, 0),
        'e':( 0, (2**0.5)/2, (2**0.5)/2, 0),
        'a':( 0,-1, 0, 0),
        'd':( 0, 1, 0, 0),
        'z':( 0,-(2**0.5)/2,-(2**0.5)/2, 0),
        'x':( 0, 0,-1, 0),
        'c':( 0, (2**0.5)/2,-(2**0.5)/2, 0),

        'i':( 1, 0, 0, 0),
        'k':(-1, 0, 0, 0),

    }

speedBindings={
        'r':(0.1,0),
        'v':(-0.1,0),
    }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/loco/command', Command, queue_size = 1)
    rospy.init_node('teleop_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    #print(speed)
    pitch = 0
    yaw = 0
    throttle = 0
    status = 0

    try:
        print(msg)
        print( "Thruster input power is currently at: " + str(speed*100) + " %")

        while(1):
            key = getKey()
            if key in moveBindings.keys():
                pitch = moveBindings[key][0]
                yaw = moveBindings[key][1]
                throttle = moveBindings[key][2]

            elif key in speedBindings.keys():
                status=status+1
                if (status == 14):
                    print(msg)
                    status = 0

                if speed+speedBindings[key][0]>1:

                    print("Thruster power at maximum.")

                elif speed+speedBindings[key][0]<0:

                    print("Thruster power at minimum.")

                else:
                    if speed + speedBindings[key][0] < 0.1:
                        speed=0

                    else:
                        speed = speed + speedBindings[key][0]

                    print( "Thruster input power is currently at: " + str(speed*100) + " %")

            else:
                pitch = 0
                yaw = 0
                throttle = 0
                if (key == '\x03'):
                    break

            command = Command()
            command.pitch=pitch*speed
            command.yaw=yaw*speed
            command.throttle=throttle*speed
            pub.publish(command)

    except Exception as e:
        print(e)

    finally:
        command = Command()
        command.pitch=0
        command.yaw=0
        command.throttle=0
        pub.publish(command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
