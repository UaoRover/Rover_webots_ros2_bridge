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


class LineFollower(Node):
    def __init__(self):
        super().__init__('linefollower_cmdvel')
        # Subscribe Infra Red sensors
        self.subs_camera = self.create_subscription(
            Image, 'img', self.camera_callback, 1)

        # Publish cmd vel
        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)

        # Initialize parameters
        self.Image_data=None

        self.cmd = Twist()

        self.cmd.linear.x = 1.0
        self.cmd.angular.z = 0.0

        self.pubs_cmdvel.publish(self.cmd)

    # Call backs to update sensor reading variables
    def camera_callback(self, msg):
        self.Image_data = msg.data



def main(args=None):

    rclpy.init(args=args)

    ls = LineFollower()
    rclpy.spin(ls)

    ls.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
