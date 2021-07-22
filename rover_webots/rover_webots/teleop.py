# Copyright 1996-2021 Soft_illusion.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import sys,tty,termios

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_cmdvel')

        self.cmd = Twist()

        while True:

            self.cmd.angular.z=0.0
            self.cmd.linear.x =0.0
            key = ord(get())
            
            print(key)
            if key==119:
                self.cmd.linear.x =1.0
            if key==115:
                self.cmd.linear.x =-1.0
            if key== 100:
                self.cmd.angular.z=-0.5
            if key==97:
                self.cmd.angular.z=0.5
            if key ==32:
                self.cmd.angular.z=0.0
                self.cmd.linear.x =0.0
            print('Vel_lin: '+str(self.cmd.linear.x)+'\nVel_ang: '+str(self.cmd.angular.z))
            self.pubs_cmdvel.publish(self.cmd)

            if key==3:
                self.cmd.angular.z=0.0
                self.cmd.linear.x =0.0
                self.pubs_cmdvel.publish(self.cmd)
                break



def main(args=None):

    rclpy.init(args=args)

    ls = Teleop()
    rclpy.spin(ls)

    ls.destroy_node()
    rclpy.shutdown()

class _Getch:       
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
            	tty.setraw(sys.stdin.fileno())
    	        ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch    

def get():
    inkey = _Getch()
    while(1):
            k=inkey()
            if k!='':break
    #print 'you pressed', ord(k)
    return k

if __name__ == '__main__':
    main()
