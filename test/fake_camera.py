#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge

rospy.init_node('fake_camera')
pub_img = rospy.Publisher('/hk_camera/image_raw/compressed', CompressedImage, queue_size=1)
pub_info = rospy.Publisher('/hk_camera/camera_info', CameraInfo, queue_size=1)
bridge = CvBridge()
rate = rospy.Rate(10)

info = CameraInfo()
info.width = 640
info.height = 480
info.distortion_model = 'plumb_bob'
info.K = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
info.header.frame_id = 'camera'

while not rospy.is_shutdown():
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (250, 140), (270, 340), (0, 0, 255), -1)
    cv2.rectangle(img, (390, 140), (410, 340), (0, 0, 255), -1)
    cv2.rectangle(img, (270, 200), (390, 280), (20, 20, 20), -1)
    msg = bridge.cv2_to_compressed_imgmsg(img, 'jpg')
    stamp = rospy.Time.now()
    msg.header.stamp = stamp
    msg.header.frame_id = 'camera'
    info.header.stamp = stamp
    pub_img.publish(msg)
    pub_info.publish(info)
    rate.sleep()
