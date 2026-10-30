# import cv2
# import rclpy
# from cv_bridge import CvBridge
# from rclpy.node import Node
# from sensor_msgs.msg import Image


# class ImagePublisher(Node):
#     def __init__(self):
#         super().__init__('img_publisher')

#         self.publisher = self.create_publisher(
#             Image,
#             '/image_raw',
#             10
#         )

#         self.bridge = CvBridge()
#         self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

#         if not self.cap.isOpened():
#             raise RuntimeError('Could not open camera device 0')

#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
#         self.cap.set(cv2.CAP_PROP_FPS, 10)

#         self.timer = self.create_timer(
#             1.0 / 10.0,
#             self.publish_frame
#         )

#         self.get_logger().info(
#             f'Video Width : {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}'
#         )
#         self.get_logger().info(
#             f'Video Height : {int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}'
#         )
#         self.get_logger().info('Video Camera Device : 0')
#         self.get_logger().info(
#             f'Video Frame Rate : {self.cap.get(cv2.CAP_PROP_FPS):.0f}'
#         )

#     def publish_frame(self):
#         ret, frame = self.cap.read()

#         if not ret:
#             self.get_logger().warning('Failed to read camera frame')
#             return

#         msg = self.bridge.cv2_to_imgmsg(
#             frame,
#             encoding='bgr8'
#         )

#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'camera'

#         self.publisher.publish(msg)

#     def destroy_node(self):
#         if self.cap.isOpened():
#             self.cap.release()

#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)

#     node = ImagePublisher()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()

#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)

        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('height', 480)
        self.height = self.get_parameter('height').value
        self.declare_parameter('camera_device', 0)
        self.camera_device = self.get_parameter('camera_device').value
        self.declare_parameter('frame_rate', 480)
        self.frame_rate = self.get_parameter('frame_rate').value

        self.cap = cv2.VideoCapture(self.camera_device)
        self.cv_bridge = CvBridge()
        time_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(time_period, self.time_callback)

        self.get_logger().info("Video Width : " + str(self.width))
        self.get_logger().info("Video Height : " + str(self.height))
        self.get_logger().info("Video Camera Device : " + str(self.camera_device))
        self.get_logger().info("Video Frame Rate : " + str(self.frame_rate))

    def time_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (self.width, self.height))
        img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(img)
        cv2.waitKey(1)

def main():
    rclpy.init() 
    node = ImgPublisher() 
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()