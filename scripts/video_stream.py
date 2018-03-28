import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class DepthCameraStreamer:
    def __init__(self, image_topic="", depth_topic="", scale=1.0):
        self.frame = None
        self.depth = None

        self.scale = scale
        self.bridge = CvBridge()

        image_sub = message_filters.Subscriber(image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 3, 0.5)
        self.ts.registerCallback(self._process)

    def _process(self, rgb_data, depth_data):
        try:
            camera_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError, e:
            print e
            return

        self.frame = cv2.resize(camera_image, (0, 0), fx=self.scale, fy=self.scale)
        self.depth = cv2.resize(depth_image, (0, 0), fx=self.scale, fy=self.scale)

    def read(self):
        return self.frame is not None and self.depth is not None, self.frame, self.depth
