#!/usr/bin/env python3

import rospy
import csv
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
from datetime import datetime
import time


#Global vars
idList = []
timeList = []
currentMarker = 999
start = time.perf_counter()


def getTime():

    #grabbing time and date to provide unique ID for logs
    dateTime = datetime.now()
    dtString = dateTime.strftime("%Y-%m-%d %H:%M:%S") #ISO 8601 Standard

    return dtString


def getPath():

    timenow = getTime()

    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    path = (packagePath + "logs/arlog_" + timenow + ".csv")

    return path



def saveCSV():
 
    with open(getPath(), "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['ID', 'Time'])
        
        for i in range(len(idList)):
            writer.writerow([idList[i], timeList[i]])


def rosInit():

    rospy.init_node("ar_logger")

    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, getTag)

    rospy.spin()
    rospy.on_shutdown()

def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True


def getTag(msg, start):
    for marker in msg.markers:
        global idList
        global currentMarker

        if marker.id != currentMarker:
            
            finish = time.perf_counter()
            elapsed = round(finish-start, 5)
            currentMarker = marker.id

            if checkDuplicate(idList, currentMarker) == True:
                continue
            else:
                timeList.append(elapsed)
                idList.append(currentMarker)

            rospy.loginfo(currentMarker)
            rospy.loginfo(elapsed)
            rospy.loginfo(idList)




if __name__ == '__main__':
    rosInit()
