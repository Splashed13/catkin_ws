#!/usr/bin/env python3

import cv2

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import depthai as dai

import numpy as np


class DepthaiCamera():
    res = [416, 416]
    fps = 20.0

    pub_topic = '/depthai_node/image/compressed'
    pub_topic_raw = '/depthai_node/image/raw'
    pub_topic_cam_inf = '/depthai_node/camera/camera_info'

    def __init__(self):
        self.pipeline = dai.Pipeline()

        # Pulbish ros image data
        self.pub_image = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=10)
        self.pub_image_raw = rospy.Publisher(self.pub_topic_raw, Image, queue_size=10)
        # Create a publisher for the CameraInfo topic
        self.pub_cam_inf = rospy.Publisher(self.pub_topic_cam_inf, CameraInfo, queue_size=10)
        # Create a timer for the callback
        self.timer = rospy.Timer(rospy.Duration(1.0 / 10), self.publish_camera_info, oneshot=False)

        rospy.loginfo("Publishing images to rostopic: {}".format(self.pub_topic))

        self.br = CvBridge()

        rospy.on_shutdown(lambda: self.shutdown())

    def publish_camera_info(self, timer=None):
        # Create a publisher for the CameraInfo topic

        # Create a CameraInfo message
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = "camera_frame"
        camera_info_msg.height = self.res[0] # Set the height of the camera image
        camera_info_msg.width = self.res[1]   # Set the width of the camera image

        # Set the camera intrinsic matrix (fx, fy, cx, cy)
        camera_info_msg.K = [615.381, 0.0, 320.0, 0.0, 615.381, 240.0, 0.0, 0.0, 1.0]
        # Set the distortion parameters (k1, k2, p1, p2, k3)
        camera_info_msg.D = [-0.10818, 0.12793, 0.00000, 0.00000, -0.04204]
        # Set the rectification matrix (identity matrix)
        camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Set the projection matrix (P)
        camera_info_msg.P = [615.381, 0.0, 320.0, 0.0, 0.0, 615.381, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        # Set the distortion model
        camera_info_msg.distortion_model = "plumb_bob"
        # Set the timestamp
        camera_info_msg.header.stamp = rospy.Time.now()

        self.pub_cam_inf.publish(camera_info_msg)  # Publish the camera info message

    def rgb_camera(self):
        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setPreviewSize(self.res[0], self.res[1])
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(self.fps)

        # Def xout / xin
        ctrl_in = self.pipeline.createXLinkIn()
        ctrl_in.setStreamName("cam_ctrl")
        ctrl_in.out.link(cam_rgb.inputControl)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")

        cam_rgb.preview.link(xout_rgb.input)

    def run(self):
        self.rgb_camera()

        with dai.Device(self.pipeline) as device:
            video = device.getOutputQueue(
                name="video", maxSize=1, blocking=False)

            while True:
                frame = video.get().getCvFrame()

                self.publish_to_ros(frame)
                self.publish_camera_info()

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.header.frame_id = "home"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.pub_image.publish(msg_out)
        # Publish image raw
        msg_img_raw = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub_image_raw.publish(msg_img_raw)
        

    def shutdown(self):
        cv2.destroyAllWindows()


def main():
    rospy.init_node('depthai_node')
    dai_cam = DepthaiCamera()

    while not rospy.is_shutdown():
        dai_cam.run()

    dai_cam.shutdown()
