#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
from ar_track_alvar_msgs.msg import AlvarMarkers

def rosInit():
     
    rospy.init_node("ar_logger")
    rospy.loginfo('init')

    var = 'show'
    rospy.loginfo(var)


    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback_ar_pose)
     

def callback_ar_pose(msg):

    rospy.loginfo('hi')
    
        

    tagPub = rospy.Publisher('tagTopic', String, queue_size=10)
    tagPub.publish('currentMarker is: ')


    for marker in msg.markers:

        rospy.loginfo('hi_2 ')

        rospy.loginfo(marker.id)
        # rospy.loginfo(msg.marker.id)

        rospy.loginfo(marker.id)

        currentMarker = marker.id

        print (currentMarker) 

        tagPub = rospy.Publisher('tagTopic', String, queue_size=10)
        tagPub.publish(currentMarker)





if __name__ == "__main__":
        
        rosInit()


       
        rospy.spin()