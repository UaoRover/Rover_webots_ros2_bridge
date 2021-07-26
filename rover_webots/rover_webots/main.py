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
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.time import Time
from rclpy.qos import QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)

        # Se define el tiempo de muestreo según la simulación
        self.service_node_vel_timestep = 32*6

        # Se crea un ROS2 service que capturará las datos de Webots
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)
        
	#-------------------------------
        #se utiliza el codigo de Webots para obtener el nombre del dispositivo https://cyberbotics.com/doc/reference/robot?tab-language=python#wb_robot_get_device
        self.camera = self.robot.getDevice(
            'camera')
        #Se habilita el respectivo dispositivo, en este caso la camara
        self.camera.enable(self.service_node_vel_timestep)

        #Se crea un publisher del sensor, con el tipo de mensaje Image y el nombre img 
        self.camera_publisher = self.create_publisher(Image, 'img', 1)
        #-------------------------------
        self.camera_left = self.robot.getDevice( 'camera_left')
        self.camera_left.enable(self.service_node_vel_timestep)
        self.camera_left_publisher = self.create_publisher( Image, 'img_left', 1)
        
        self.camera_right = self.robot.getDevice('camera_right') 
        self.camera_right.enable(self.service_node_vel_timestep)
        self.camera_right_publisher = self.create_publisher( Image, 'img_right', 1)
        
        self.gps = self.robot.getDevice('gps') 
        self.gps.enable(self.service_node_vel_timestep)
        self.gps_publisher = self.create_publisher( NavSatFix, 'gps', 1)
        #-------------------------------
        self.acel = self.robot.getDevice( 'accelerometer')
        self.gyro = self.robot.getDevice('gyro')
        self.iu = self.robot.getDevice('imu')
        self.acel.enable(self.service_node_vel_timestep)
        self.gyro.enable(self.service_node_vel_timestep)
        self.iu.enable(self.service_node_vel_timestep)
        self.imu_publisher = self.create_publisher( Imu, 'imu', 1)
        #-------------------------------
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.service_node_vel_timestep)
        self.lidar_publisher = self.create_publisher(LaserScan, 'Lidar', 1)
        #-------------------------------
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
        camera_data_left = self.camera_left.getImage()
        camera_data_right = self.camera_right.getImage()


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
        #-----------------------------------
        msg_left = Image()
        msg_left.height = self.camera_left.getHeight()
        msg_left.width = self.camera_left.getWidth()
        msg_left.is_bigendian = False
        msg_left.step = self.camera_left.getWidth() * 4
        msg_left.header.frame_id = 'camera_left'
        msg_left._data = camera_data_left
        msg_left.encoding = 'bgra8'
        #se publica el mensaje
        self.camera_left_publisher.publish(msg_left)
        #-----------------------------------
        msg_right = Image()
        msg_right.height = self.camera_right.getHeight()
        msg_right.width = self.camera_right.getWidth()
        msg_right.is_bigendian = False
        msg_right.step = self.camera_right.getWidth() * 4
        msg_right.header.frame_id = 'camera_right'
        msg_right._data = camera_data_right
        msg_right.encoding = 'bgra8'
        self.camera_right_publisher.publish(msg_right)
        #-----------------------------------
        msg_gps = NavSatFix()
        msg_gps.header.stamp = self.get_clock().now().to_msg()
        msg_gps.header.frame_id = 'gps'
        msg_gps.latitude = self.gps.getValues()[0]*0.00001
        msg_gps.longitude = self.gps.getValues()[2]*0.00001
        msg_gps.altitude = self.gps.getValues()[1]
        msg_gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg_gps.status.service = NavSatStatus.SERVICE_GPS
        msg_gps.status.status = NavSatStatus.STATUS_FIX
        self.gps_publisher.publish(msg_gps)
        #----------------------------------------------------------------------------------
        #----------------------------------------------------------------------------------
        msg_imu = Imu()
        msg_imu.header.stamp = self.get_clock().now().to_msg()
        msg_imu.header.frame_id = 'imu'
        gyro_data = self.gyro.getValues()
        msg_imu.angular_velocity.x = interpolate_lookup_table(gyro_data[0], self.gyro.getLookupTable())
        msg_imu.angular_velocity.y = interpolate_lookup_table(gyro_data[1], self.gyro.getLookupTable())
        msg_imu.angular_velocity.z = interpolate_lookup_table(gyro_data[2], self.gyro.getLookupTable())
        acel_data =self.acel.getValues()
        msg_imu.linear_acceleration.x = interpolate_lookup_table(acel_data[0], self.acel.getLookupTable())
        msg_imu.linear_acceleration.y = interpolate_lookup_table(acel_data[1], self.acel.getLookupTable())
        msg_imu.linear_acceleration.z = interpolate_lookup_table(acel_data[2], self.acel.getLookupTable())
        iu_data = self.iu.getQuaternion()
        msg_imu.orientation.x = iu_data[0]
        msg_imu.orientation.y = iu_data[1]
        msg_imu.orientation.z = iu_data[2]
        msg_imu.orientation.w = iu_data[3]
        self.imu_publisher.publish(msg_imu)
        #----------------------------------------------------------------------------------
        ranges = self.lidar.getLayerRangeImage(0)
        msg_lidar = LaserScan()
        msg_lidar.header.stamp = self.get_clock().now().to_msg()
        msg_lidar.header.frame_id = 'lidar'
        msg_lidar.angle_min = -0.5 * self.lidar.getFov()
        msg_lidar.angle_max = 0.5 *self.lidar.getFov()
        msg_lidar.angle_increment = self.lidar.getFov() / (self.lidar.getHorizontalResolution() -1)
        msg_lidar.scan_time = self.lidar.getSamplingPeriod() / 1000
        msg_lidar.range_min = self.lidar.getMinRange() 
        msg_lidar.range_max = self.lidar.getMaxRange() 
        msg_lidar.ranges = ranges
        self.lidar_publisher.publish(msg_lidar)

def main(args=None):
    rclpy.init(args=args)
    
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
