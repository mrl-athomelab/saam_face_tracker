#!/usr/bin/env python

import rospy
from face_detector import LiveFaceDetector
from dynamixel import DynamixelMotor
import sys
import random


class FaceTracker:
    def __init__(self, speed, motor_id):
        rospy.loginfo("Connecting to face detector service ...")
        self.detector = LiveFaceDetector(service='/ros_face_recognition/faces')

        self.speed = speed
        rospy.loginfo("Connecting to motor ...")
        self.motor = DynamixelMotor(motor_id)
        self.motor.set_speed(self.speed)
        direction = random.random()
        if direction < 0.5:
            direction = 1
        else:
            direction = -1
        self.motor.direction = direction

    def run(self, face_label):
        try:
            image_h, image_w = (200, 200)
            faces = self.detector.detect((image_h, image_w))

            human_detected = None
            for face in faces:
                if face.name == face_label:
                    human_detected = face

            if human_detected is None:
                position = self.motor.get_position()
                if position > 45:
                    self.motor.direction = -1
                elif position < -45:
                    self.motor.direction = 1
                self.motor.set_speed(self.speed * self.motor.direction)
            else:
                distance = (image_w / 2) - (human_detected.x + human_detected.w / 2)
                speed = distance / 3

                if distance < 0:
                    self.motor.direction = -1
                else:
                    self.motor.direction = 1

                self.motor.set_speed(speed)
        except KeyboardInterrupt:
            rospy.logwarn("Shutting done ...")
            return False
        except rospy.service.ServiceException:
            rospy.logwarn("Shutting done ...")
            return False


if __name__ == "__main__":
    rospy.init_node("saam_face_tracker", anonymous=True)

    if len(sys.argv) != 2:
        print "Please enter face label as argument"
        sys.exit(1)

    tracker = FaceTracker(speed=25, motor_id=6)
    while True:
        if tracker.run(sys.argv[1]) is not None:
            break

    tracker.motor.set_speed(0)
    rospy.signal_shutdown("keyboard interrupt")
