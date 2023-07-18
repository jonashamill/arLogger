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
    with open(getPath())


def rosInit():

    rospy.init_node("ar_logger")

    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback_ar_pose)

    rospy.spin()
    rospy.on_shutdown()

def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True


def getTag(msg):
    for marker in msg.markers:
        global idList
        global currentMarker



def main():

    

    start = time.perf_counter()

    finish = time.perf_counter()

    elapsed = round(finish-start, 5)







if __name__ == '__main__':
    rosInit()
    main()