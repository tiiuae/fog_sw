import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

from sensor_msgs.msg import Image

class VideoSubscriber(Node):

    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.image = None
        self.br = CvBridge()

    def listener_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("Drone", self.image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    video_subscriber = VideoSubscriber()

    rclpy.spin(video_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()

    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
