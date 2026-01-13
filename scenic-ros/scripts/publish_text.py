"""
    @Author Jason Hughes
    @Date January 2025

    @About A quick script to publish text
"""

import time
import rclpy
from rclpy.node import Node
from scenic_msgs.msg import Text, TextArray

class TextPublisherNode(Node):

    def __init__(self):
        super().__init__('text_publisher_node')
        self.pub_ = self.create_publisher(TextArray, "/text", 1)

        time.sleep(0.5)

        msg = TextArray()
        regmsg = Text() 

        regmsg.label = "road"
        regmsg.level = 1
        regmsg.priority = 0

        objmsg = Text()

        objmsg.label = "car"
        objmsg.level = 0 
        objmsg.priority = 3

        msg.classes = [regmsg, objmsg]

        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TextPublisherNode()

    time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
