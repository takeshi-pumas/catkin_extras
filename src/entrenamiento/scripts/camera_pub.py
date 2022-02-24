#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
	pubImage = rospy.Publisher('/web_cam', Image, queue_size=10)
	rospy.init_node('web_cam_publisher', anonymous=True)
	cap = cv2.VideoCapture(0)
	rate = rospy.Rate(60)
	bridge = CvBridge()
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		pubImage.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
		#print(frame.shape)
		cv2.imshow('WebCam',frame)
		cv2.waitKey(1)
		rate.sleep()
		
	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass