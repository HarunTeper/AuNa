#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header



class TF_Converter(Node):

    def __init__(self):
        super().__init__('tf_converter')
        self.declare_parameter('namespace', 'default')
        self.subscription = self.create_subscription(TFMessage,'/tf',self.listener_callback,10)
        self.publisher_ = self.create_publisher(TFMessage,'tf', 10)

    def listener_callback(self, msg):
        for transform in msg.transforms:
            header = transform.header
            frame_id = header.frame_id
            frame_id_parts = frame_id.split('/')
            if frame_id_parts[0]== self.get_parameter('namespace').get_parameter_value().string_value:
                self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    tf_converter_node = TF_Converter()

    rclpy.spin(tf_converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
