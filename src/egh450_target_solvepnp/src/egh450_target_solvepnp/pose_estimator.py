
#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Time 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray  # Import for the corners topic



class PoseEstimator():
    def __init__(self):
        self.broadcaster = Tf2BroadcasterTarget()
        self.corners = None
        
        self.pub_aruco = rospy.Publisher('/emulated_uav/aruco', Float32MultiArray, queue_size=10)
        self.tvec_dict = {}
        self.aruco_detections = [] 
        
        # Set up the CV Bridge
        self.bridge = CvBridge()

        # Load in parameters from ROS
        self.param_use_compressed = rospy.get_param("~use_compressed", False)
        self.param_marker_size = rospy.get_param("~marker_size", 0.02)
        self.param_hue_center = rospy.get_param("~hue_center", 170)
        self.param_hue_range = rospy.get_param("~hue_range", 20) / 2
        self.param_sat_min = rospy.get_param("~sat_min", 50)
        self.param_sat_max = rospy.get_param("~sat_max", 255)
        self.param_val_min = rospy.get_param("~val_min", 50)
        self.param_val_max = rospy.get_param("~val_max", 255)

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
        self.sub_corners = rospy.Subscriber("/aruco_corners", Float32MultiArray, self.corners_callback)

        # Generate the model for the pose solver
        marker = self.param_marker_size
        self.model_object = np.array([
                                      (-marker, marker, 0.0),
                                      (marker, marker, 0.0),
                                      (marker, -marker, 0.0),
                                      (-marker, -marker, 0.0)], dtype=np.float32)


    def shutdown(self):
        # Unregister anything that needs it here
        self.sub_info.unregister()
        self.sub_img.unregister()
        self.sub_corners.unregister()

    # Collect in the camera characteristics
    def callback_info(self, msg_in):
        self.dist_coeffs = np.array([-0.10818, 0.12793, 0.00000, 0.00000, -0.04204], dtype=np.float32)
        
        self.camera_matrix = np.array([(615.381, 0.0, 320.0), 
                                        (0.0, 615.381, 240.0),
                                        (0.0, 0.0, 1.0)], dtype=np.float32)

		

        self.got_camera_info = True

    def corners_callback(self, msg):

        # Extract the corners and ID from the message
        data = msg.data
        marker_id = int(data[-1])  # The last element is the marker ID
        self.corners = data  # The rest are the corner coordinates
       
	
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
                if(self.corners):
                    self.model_image = np.array([
                                                
                                                (self.corners[0], self.corners[1]),
                                                (self.corners[2], self.corners[3]),
                                                (self.corners[4], self.corners[5]),
                                                (self.corners[6], self.corners[7])])
                    rospy.loginfo(self.model_image)

                    # Do the SolvePnP method
                    (success, rvec, tvec) = cv2.solvePnP(self.model_object, self.model_image, self.camera_matrix, self.dist_coeffs)
                    rospy.loginfo(success)
                    # If a result was found, send to TF2
                    if success:

                        msg_out = TransformStamped()
                        msg_out.header = msg_in.header
                        msg_out.child_frame_id = "circle"
                        msg_out.transform.translation.x = tvec[0]
                        msg_out.transform.translation.y = tvec[1]
                        msg_out.transform.translation.z = tvec[2]
                        msg_out.transform.rotation.w = 1.0	# Could use rvec, but need to convert from DCM to quaternion first
                        msg_out.transform.rotation.x = 0.0
                        msg_out.transform.rotation.y = 0.0
                        msg_out.transform.rotation.z = 0.0

                        self.broadcaster.send_tf_target(tvec[0], tvec[1], tvec[2])

                        marker_id = int(self.corners[-1])  # Get the marker ID (assume it's passed in corners)
                        self.tvec_dict[marker_id] = tvec.flatten()

                        # Convert the dictionary to a two-column array
                        aruco_array = Float32MultiArray()
                        for marker_id, tvec_value in self.tvec_dict.items():
                            aruco_array.data.extend([marker_id, tvec_value[0], tvec_value[1], tvec_value[2]])

                        self.pub_aruco.publish(aruco_array)

                        rospy.loginfo(aruco_array)
    
                        rospy.loginfo(tvec)

# Use this to broadcoast pose estimation
class Tf2BroadcasterTarget:
    def __init__(self, camera_name="camera", target_name="target"):
        self.camera_name = camera_name
        self.target_name = target_name
        
        rospy.loginfo("tf2_broadcaster_target sending target found...")

        # Setup tf2 broadcaster and timestamp publisher
        self.tfbr = tf2_ros.TransformBroadcaster()
        self.pub_found = rospy.Publisher('/emulated_uav/target_found', Time, queue_size=10)


    def send_tf_target(self, estimate_x=-0.4, estimate_y=0.2, estimate_z=1.35):
        # Generate our "found" timestamp
        time_found = rospy.Time.now()

        # Create a transform arbitrarily in the camera frame
        t = TransformStamped()
        t.header.stamp = time_found
        t.header.frame_id = self.camera_name
        t.child_frame_id = self.target_name
        
        t.transform.translation.x = estimate_x
        t.transform.translation.y = estimate_y
        t.transform.translation.z = estimate_z  # - altitude of the UAV camera Z.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transformation to TF and publish the "found" timestamp
        self.tfbr.sendTransform(t)
        self.pub_found.publish(time_found)
        rospy.loginfo(f'tf2_broadcaster_target sent TF: {t.transform.translation} and timestamp: {time_found}')
              
        

def main():
    rospy.init_node('pose_estimator', anonymous=True)
    estimator = PoseEstimator()
    rospy.spin()

if __name__ == '__main__':
    main()
