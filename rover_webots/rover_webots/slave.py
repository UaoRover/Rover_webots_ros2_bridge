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
from webots_ros2_core.webots_node import WebotsNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)

        # Se define el tiempo de muestreo según la simulación
        self.service_node_vel_timestep = 32*6

        # Se crea un ROS2 service que capturará las datos de Webots
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)

        #se utiliza el codigo de Webots para obtener el nombre del dispositivo https://cyberbotics.com/doc/reference/robot?tab-language=python#wb_robot_get_device
        self.camera = self.robot.getDevice(
            'camera')
        #Se habilita el respectivo dispositivo, en este caso la camara
        self.camera.enable(self.service_node_vel_timestep)

        #Se crea un publisher del sensor, con el tipo de mensaje Image y el nombre img 
        self.camera_publisher = self.create_publisher(
            Image, 'img', 1)

        #mensaje en terminal
        self.get_logger().info('Sensor enabled')

        # Se obtienen los nombes de los motores, de pone su posición en infinito y su velocidad en 0 justo como en Webots
        self.boggie_left_motor = self.robot.getDevice('wheel_boggie_left')
        self.boggie_left_motor.setPosition(float('inf'))
        self.boggie_left_motor.setVelocity(0)

        self.middle_left_motor = self.robot.getDevice('middle_wheel_left')
        self.middle_left_motor.setPosition(float('inf'))
        self.middle_left_motor.setVelocity(0)

        # Rear wheels
        self.rocker_left_motor = self.robot.getDevice('wheel_rocker_left')
        self.rocker_left_motor.setPosition(float('inf'))
        self.rocker_left_motor.setVelocity(0)

        self.boggie_right_motor = self.robot.getDevice('wheel_boggie_der')
        self.boggie_right_motor.setPosition(float('inf'))
        self.boggie_right_motor.setVelocity(0)


        self.middle_right_motor = self.robot.getDevice('middle_wheel_der')
        self.middle_right_motor.setPosition(float('inf'))
        self.middle_right_motor.setVelocity(0)


        self.rocker_right_motor = self.robot.getDevice('wheel_rocker_der')
        self.rocker_right_motor.setPosition(float('inf'))
        self.rocker_right_motor.setVelocity(0)


        self.left_boggie_directional_motor = self.robot.getDevice('left_boggie_directional')
        self.left_boggie_directional_motor.setPosition(float('inf'))
        self.left_boggie_directional_motor.setVelocity(0)

        self.left_rocker_directional_motor = self.robot.getDevice('left_rocker_directional')
        self.left_rocker_directional_motor.setPosition(float('inf'))
        self.left_rocker_directional_motor.setVelocity(0)

        self.der_rocker_directional_motor = self.robot.getDevice('der_rocker_directional')
        self.der_rocker_directional_motor.setPosition(float('inf'))
        self.der_rocker_directional_motor.setVelocity(0)

        self.der_boggie_directional_motor = self.robot.getDevice('der_boggie_directional')
        self.der_boggie_directional_motor.setPosition(float('inf'))
        self.der_boggie_directional_motor.setVelocity(0)                        
        #Se obtiene la velocidad maxima de uno de los motores de Webots como referencia
        self.motor_max_speed = self.rocker_right_motor.getMaxVelocity()

        #Se crea un Suscriber del comando de velocidad con el tipo de mensaje Twist y de nombre cmd_vel, se pone un callback para que se ejecute cuando recibe el mensaje
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

    #callback del suscriber
    def cmdVel_callback(self, msg):
        #calculo de Cinematica
        #wheel_gap = 0.1  # in meter
        #wheel_radius = 0.04  # in meter

        #left_speed = ((2.0 * msg.linear.x - msg.angular.z *
        #               wheel_gap) / (2.0 * wheel_radius))
        #right_speed = ((2.0 * msg.linear.x + msg.angular.z *
        #                wheel_gap) / (2.0 * wheel_radius))
        #left_speed = min(self.motor_max_speed,
        #                 max(-self.motor_max_speed, left_speed))
        #right_speed = min(self.motor_max_speed,
        #                  max(-self.motor_max_speed, right_speed))
        #se envia la velocidad a las llantas
        self.boggie_left_motor.setVelocity(msg.linear.x)
        self.middle_left_motor.setVelocity(msg.linear.x)
        self.rocker_left_motor.setVelocity(msg.linear.x)
        self.boggie_right_motor.setVelocity(msg.linear.x)
        self.middle_right_motor.setVelocity(msg.linear.x)
        self.rocker_right_motor.setVelocity(msg.linear.x)
        self.left_boggie_directional_motor.setVelocity(-msg.angular.z)
        self.left_rocker_directional_motor.setVelocity(msg.angular.z)
        self.der_rocker_directional_motor.setVelocity(msg.angular.z)
        self.der_boggie_directional_motor.setVelocity(-msg.angular.z)

    #callback del service
    def sensor_callback(self):

        #Se obtiene la imagen y se la da a la información del mensaje
        camera_data = self.camera.getImage()
        
        # Se crea un objeto del mensaje Image
        msg = Image()
        msg.height = self.camera.getHeight()
        msg.width = self.camera.getWidth()
        msg.is_bigendian = False
        msg.step = self.camera.getWidth() * 4
        msg.header.frame_id = 'camera'
        msg._data = camera_data
        msg.encoding = 'bgra8'

        #se publica el mensaje
        self.camera_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
