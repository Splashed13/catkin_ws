
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import subprocess

def id_callback(data):
    # Check if the ID is detected
    if data.data >= 0:
        # Log the numeric data received
        rospy.loginfo("Received numeric data: %d", data.data)
        
        # Use eSpeak to synthesize the speech
        subprocess.run(['espeak', f'The number detected is {data.data}'])
    else:
        # Log that the ID is not found
        rospy.loginfo("ID not found")
        
        # Use eSpeak to synthesize the speech for ID not found
        subprocess.run(['espeak', 'ID not found'])

def main():
    rospy.init_node('espeak_subscriber', anonymous=True)
    rospy.Subscriber('/aruco_id', Int32, id_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
    





