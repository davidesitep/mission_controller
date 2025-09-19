# Mission sender node

#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

MISSION_FILE_PATH = "/home/davide/Scrivania/MTCOM/mission_control/mission_file.gpx"

def mission_file_sender():
    pub = rospy.Publisher('/mission_file', String, queue_size=10)
    rospy.init_node('mission_file_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10Hz

    with open(MISSION_FILE_PATH, 'r') as file:
        gpx_content = file.read()
        
    if not rospy.is_shutdown():
        rospy.loginfo("Invio traccia GPX...")
        # Scorro i waypoints del file GPX e li invio uno ad uno al subscriber del mission_file_handler
        pub.publish(gpx_content)
        rospy.loginfo("Traccia GPX inviata.")


# MAIN
if __name__ == '__main__':
    try: 
        mission_file_sender()
    except rospy.ROSInterruptException:
        pass

