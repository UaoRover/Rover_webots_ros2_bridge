#!/usr/bin/env python 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher_vel(Node):

    def __init__(self):
        super().__init__('publisher_vel')#nombre del nodo
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)#tipo de mensaje, nombre del topic, buffer size(cantidad de mensajes)
        timer_period = 0.5  # corre el scrpit cada 0.5 segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()#creamos el mensaje con el objeto Twist
        msg.linear.x=0.1#se pone los valores al mensaje
        self.publisher_.publish(msg)# se publica


def main(args=None):
    rclpy.init(args=args)

    publisher_obj=Publisher_vel()

    rclpy.spin(publisher_obj)

    publisher_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
