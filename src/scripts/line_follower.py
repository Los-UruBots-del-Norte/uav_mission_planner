#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('line_follower')
        # Create a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()
        # Subscribe to the camera image topic
        # self.image_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        processed_image = self.process_image(cv_image)
        self.follow_line(processed_image)
        # Display the result
        cv2.imshow("Line Following", processed_image)
        cv2.waitKey(3)

    # def process_image(self, cv_image):
    #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #     _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    #     edges = cv2.Canny(thresholded, 50, 150)
    #     return edges
    
    # def process_image(self, cv_image):
    #     # Convert to grayscale
    #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
    #     # Threshold the image to isolate white lines
    #     _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    #     # Find edges for better line detection
    #     edges = cv2.Canny(thresholded, 50, 150)

    #     # Optionally, apply Hough Line Transform to find lines
    #     lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
    #     if lines is not None:
    #         for line in lines:
    #             x1, y1, x2, y2 = line[0]
    #             cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

    #     return cv_image

    def follow_line(self, processed_image):
        lines = cv2.HoughLinesP(processed_image, 1, np.pi/180, 50, None, 50, 10)
        cmd_vel = Twist()
        if lines is not None:
            cmd_vel.linear.x = 0.5  # Example forward speed
            cmd_vel.angular.z = 0.1  # Example angular speed
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0


def main():
    try:
        lf = LineFollower()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()