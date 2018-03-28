import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ros_face_recognition.srv import Detect, Face


class FaceDetector:
    def __init__(self, service_name):
        self.bridge = CvBridge()
        rospy.wait_for_service(service_name)
        self.service = rospy.ServiceProxy(service_name, Detect)

    def detect(self, image, shape=None):
        encoded_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        resp = self.service(input_image=encoded_image)

        image_h, image_w = image.shape[:2]
        if shape is not None:
            image_h, image_w = shape

        faces = resp.faces
        for index in range(len(faces)):
            f = faces[index]
            faces[index].x = int(f.x * image_w)
            faces[index].y = int(f.y * image_h)
            faces[index].w = int(f.w * image_w)
            faces[index].h = int(f.h * image_h)

        return faces


class LiveFaceDetector:
    def __init__(self, service):
        rospy.wait_for_service(service)
        self.service = rospy.ServiceProxy(service, Face)
        self.bridge = CvBridge()

    def detect(self, shape):
        image_h, image_w = shape

        resp = self.service()
        faces = resp.faces
        for index in range(len(faces)):
            f = faces[index]
            faces[index].x = int(f.x * image_w)
            faces[index].y = int(f.y * image_h)
            faces[index].w = int(f.w * image_w)
            faces[index].h = int(f.h * image_h)

        return faces