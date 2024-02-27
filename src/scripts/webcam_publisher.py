#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:

    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.pub = rospy.Publisher('/camera_1', Image, queue_size=10)
        self.rgb8pub = rospy.Publisher('/camera_1/rgb', Image, queue_size=10)
        self.bgr8pub = rospy.Publisher('/camera_1/bgr', Image, queue_size=10)
        self.mono8pub = rospy.Publisher('/camera_1/mono', Image, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            try:
                r, frame = self.cap.read()
                if not r:
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                # BGR8
                self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                # RGB8
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.rgb8pub.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8"))
                # MONO8
                frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.mono8pub.publish(self.bridge.cv2_to_imgmsg(frame_mono, "mono8"))

            except CvBridgeError as e:
                print(e)
                self.cap.release()

def main(args=None):
    ip = ImagePublisher()
    print("Publishing image from webcam...")
    ip.run()

if __name__ == '__main__':
    main()