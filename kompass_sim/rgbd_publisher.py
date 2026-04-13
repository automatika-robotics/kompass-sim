"""Minimal node to combine color/depth images into a realsense2_camera_msgs/RGBD message."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD


class RGBDPublisher(Node):
    def __init__(self):
        super().__init__("rgbd_publisher")

        self.declare_parameter("rgb_image_topic", "/color_camera/image_raw/image_color")
        self.declare_parameter("depth_image_topic", "/depth_camera/image_raw/image")
        self.declare_parameter("rgb_camera_info_topic", "/color_camera/image_raw/camera_info")
        self.declare_parameter("depth_camera_info_topic", "/depth_camera/image_raw/camera_info")
        self.declare_parameter("rgbd_topic", "/camera/rgbd")
        self.declare_parameter("publish_rate", 10.0)

        rgb_topic = self.get_parameter("rgb_image_topic").value
        depth_topic = self.get_parameter("depth_image_topic").value
        rgb_info_topic = self.get_parameter("rgb_camera_info_topic").value
        depth_info_topic = self.get_parameter("depth_camera_info_topic").value
        rgbd_topic = self.get_parameter("rgbd_topic").value
        publish_rate = self.get_parameter("publish_rate").value

        self._latest_rgb = None
        self._latest_depth = None
        self._latest_rgb_info = None
        self._latest_depth_info = None

        self.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, rgb_info_topic, self._rgb_info_cb, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self._depth_info_cb, 10)

        self._pub = self.create_publisher(RGBD, rgbd_topic, 10)
        self.create_timer(1.0 / publish_rate, self._publish_rgbd)

        self.get_logger().info(
            f"RGBD publisher ready: subscribing to '{rgb_topic}', '{depth_topic}', "
            f"publishing on '{rgbd_topic}'"
        )

    def _rgb_cb(self, msg):
        self._latest_rgb = msg

    def _depth_cb(self, msg):
        self._latest_depth = msg

    def _rgb_info_cb(self, msg):
        self._latest_rgb_info = msg

    def _depth_info_cb(self, msg):
        self._latest_depth_info = msg

    def _publish_rgbd(self):
        if self._latest_rgb is None or self._latest_depth is None:
            return
        rgbd = RGBD()
        rgbd.header = self._latest_rgb.header
        if self._latest_rgb_info is not None:
            rgbd.rgb_camera_info = self._latest_rgb_info
        if self._latest_depth_info is not None:
            rgbd.depth_camera_info = self._latest_depth_info
        rgbd.rgb = self._latest_rgb
        rgbd.depth = self._latest_depth
        self._pub.publish(rgbd)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
