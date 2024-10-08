
#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

from std_msgs.msg import Time 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray  # Import for the corners topic
from custom_msgs.msg import Detection



class PoseEstimator():
    def __init__(self):
        self.broadcaster = Tf2BroadcasterTarget()
        self.corners = None
        self.tvec_dict = {}
        self.model_object = None
        self.target = None
        self.pub_aruco = rospy.Publisher('/emulated_uav/target', Float32MultiArray, queue_size=10)

        
        # Set up the CV Bridge
        self.bridge = CvBridge()

        # Load in parameters from ROS
        self.param_use_compressed = rospy.get_param("~use_compressed", False)
        self.param_marker_size_human = rospy.get_param("~marker_size", 0.1)
        self.param_marker_size_bag = rospy.get_param("~marker_size", 0.1)

        # Set additional camera parameters
        self.got_camera_info = False
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set up the publishers, subscribers, and tf2
        self.sub_info = rospy.Subscriber("~camera_info", CameraInfo, self.callback_info)

        if self.param_use_compressed:
            self.sub_img = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.callback_img)
            self.pub_mask = rospy.Publisher("~debug/image_raw/compressed", CompressedImage, queue_size=1)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw/compressed", CompressedImage, queue_size=1)
        else:
            self.sub_img = rospy.Subscriber("~image_raw", Image, self.callback_img)
            self.pub_mask = rospy.Publisher("~debug/image_raw", Image, queue_size=1)
            self.pub_overlay = rospy.Publisher("~overlay/image_raw", Image, queue_size=1)

        self.tfbr = tf2_ros.TransformBroadcaster()

        # New subscriber for the /aruco_corners topic
        self.sub_target = rospy.Subscriber("/depthai_node/bounding_boxes", Float32MultiArray, self.target_callback)

        # Generate the model for the pose solver
        marker1 = self.param_marker_size_human
        marker2 = self.param_marker_size_bag
        self.model_object1 = np.array([
                                      (-marker1, marker1, 0.0), #TL
                                      (marker1, marker1, 0.0),  #TR
                                      (marker1, -marker1, 0.0), #BR
                                      (-marker1, -marker1, 0.0)]) #BL
        self.model_object2 = np.array([
                                      (-marker2, marker2, 0.0),
                                      (marker2, marker2, 0.0),
                                      (marker2, -marker2, 0.0),
                                      (-marker2, -marker2, 0.0)])
        
        

    def shutdown(self):
        # Unregister anything that needs it here
        self.sub_info.unregister()
        self.sub_img.unregister()
        self.sub_target.unregister()

    # Collect in the camera characteristics
    def callback_info(self, msg_in):
        self.dist_coeffs = np.array([-0.10818, 0.12793, 0.00000, 0.00000, -0.04204], dtype=np.float32)
        
        self.camera_matrix = np.array([(615.381, 0.0, 320.0), 
                                        (0.0, 615.381, 240.0),
                                        (0.0, 0.0, 1.0)], dtype=np.float32)


        self.got_camera_info = True

    def target_callback(self, msg):

        # Extract the corners and ID from the message
        data = msg.data
        
        self.target = data  # The rest are the corner coordinates
 
	
    def callback_img(self, msg_in):
            
            # Don't bother to process image if we don't have the camera calibration
            if self.got_camera_info:
                #Convert ROS image to CV image
                cv_image = None

                try:
                    if self.param_use_compressed:
                        cv_image = self.bridge.compressed_imgmsg_to_cv2( msg_in, "bgr8" )
                    else:
                        cv_image = self.bridge.imgmsg_to_cv2( msg_in, "bgr8" )
                except CvBridgeError as e:
                    rospy.loginfo(e)
                    return


				# Calculate the pictured the model for the pose solver
				# For this example, draw a square around where the circle should be
				# There are 5 points, one in the center, and one in each corner
                if(self.target):
                    self.model_image = np.array([
                                                (self.target[1]*416, self.target[3]*416), #TR
                                                (self.target[0]*416, self.target[3]*416),  #TL
                                                (self.target[0]*416, self.target[2]*416), #BL
                                                (self.target[1]*416, self.target[2]*416) #BR
                                                ])


                    # Do the SolvePnP method
                    class_id = int(self.target[-1])  # Get the marker ID (assume it's passed in corners)
                    class_str = None
                    if (class_id == 0):
                        self.model_object = self.model_object1
                        class_str = "bag"
                    elif (class_id == 4):
                        self.model_object = self.model_object2
                        class_str = "man"
                    

                    if (class_str):
                        (success, rvec, tvec) = cv2.solvePnP(self.model_object, self.model_image, self.camera_matrix, self.dist_coeffs)
        
                        # If a result was found, send to TF2
                        if success:
                            self.broadcaster.send_tf_target(tvec[0], tvec[1], tvec[2], class_str) # send 4th paramater 'detection_type' (string)
                            success = False
                            self.target = []
                            class_str = None

                        
    



class Tf2BroadcasterTarget:
    def __init__(self, camera_name="camera", target_name="target"):
        self.camera_name = camera_name
        self.target_name = target_name
        
        rospy.loginfo("tf2_broadcaster_target sending target found...")

        # Setup tf2 broadcaster and timestamp publisher
        self.tfbr = tf2_ros.TransformBroadcaster()
        self.pub_found = rospy.Publisher('/uav/target_found', Detection, queue_size=10)

    def send_tf_target(self, estimate_x=-0.4, estimate_y=0.2, estimate_z=1.35, detection_type="man"):
        if detection_type is None:
            rospy.logwarn("No detection type specified, cannot send 'target detected' message")
            return 

        # Generate our "found" timestamp
        time_found = rospy.Time.now()

        detection_msg = Detection()
        detection_msg.detection_time = Time()
        detection_msg.detection_time.data = time_found  # Set current time 
        detection_msg.object_name = detection_type

        # Initialize the pose field (PoseStamped) with zeros, as it will be overwritten        
        detection_msg.pose = PoseStamped()
        detection_msg.pose.header.stamp = time_found

        detection_msg.pose.pose.position.x = 0.0
        detection_msg.pose.pose.position.y = 0.0
        detection_msg.pose.pose.position.z = 0.0
        detection_msg.pose.pose.orientation.x = 0.0
        detection_msg.pose.pose.orientation.y = 0.0
        detection_msg.pose.pose.orientation.z = 0.0
        detection_msg.pose.pose.orientation.w = 1.0

        # Create a transform in the camera frame based on provided estimates
        t = TransformStamped()
        t.header.stamp = time_found
        t.header.frame_id = self.camera_name
        t.child_frame_id = self.target_name
        t.transform.translation.x = estimate_x
        t.transform.translation.y = estimate_y
        t.transform.translation.z = estimate_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transformation to TF and publish the "found" timestamp
        self.tfbr.sendTransform(t)
        self.pub_found.publish(detection_msg)
        rospy.loginfo(f'tf2_broadcaster_target sent detection msg- detection type: {detection_type}, pose: {t.transform.translation}')

def main():
    rospy.init_node('pose_estimator', anonymous=True)
    estimator = PoseEstimator()
    rospy.spin()

if __name__ == '__main__':
    main()