#!/usr/bin/env python3

import rospy
import csv
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
from datetime import datetime
import time
import os
from std_msgs.msg import Int32, Float32

#Global vars
idList = []
timeList = []
lastTimestamp={}
timeSinceList = []
minList = []
maxList = []
timeTaken = 0
currentMarker = 999
start = time.perf_counter()
maxVel = 0
minVel = 0


def getTime():

    #grabbing time and date to provide unique ID for logs
    dateTime = datetime.now()
    dtString = dateTime.strftime("%Y%m%d%H%M%S") #ISO 8601 Standard

    return dtString


def getPath():

    timenow = getTime()

    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    path = os.path.join(packagePath, "logs")

    fullpath = os.path.join(path, "arlog_" + timenow + ".csv")

    print (fullpath)

    return path, fullpath

def makeFolder():

    path, _ = getPath()

    testFile = None

    # test folder permisions
    try:
        testFile = open(os.path.join(path, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(path)
        except OSError:
            print("No log folder created")
        else:
            print("Log folder created")

    testFile.close()
    os.remove(testFile.name)


def saveCSV():
    
    _, filename = getPath()

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['ID', 'Time', 'Timesince', 'Min Velocity', 'Max Velocity'])
        
        for i in range(len(idList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], minList[i], maxList[i]])


def rosInit():

    rospy.init_node("arLogger")


    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, getTag)

    rospy.Subscriber('maxVelocity', Float32, maxVelocityCallback)
    rospy.Subscriber('minVelocity', Float32, minVelocityCallback)

    
    rospy.on_shutdown(saveCSV)


def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True


def maxVelocityCallback(msg):

    global maxVel

    maxVel = msg.data



def minVelocityCallback(msg):

    global minVel

    minVel = msg.data




def getTag(msg):

    for marker in msg.markers:
        global idList
        global currentMarker
        global timeTaken
        global lastTimestamp

        if marker.id != currentMarker:
            
            finish = time.perf_counter()

            if currentMarker == 999:  # Check if timeList is empty
                timeSinceLast = 0
            else:
                timeSinceLast = round(finish - timeList[-1], 5)

            #timeSinceLast = round(finish - lastTimestamp.get(marker.id, finish), 5)
            lastTimestamp[marker.id] = finish
            timeTaken = round(finish-start, 5)
            currentMarker = marker.id
            

            

            if checkDuplicate(idList, currentMarker) == True:
                continue
            else:
                minList.append(minVel)
                maxList.append(maxVel)
                timeList.append(timeTaken)
                idList.append(currentMarker)
                timeSinceList.append(timeSinceLast)
            

            timeSince(timeSinceLast)


            rospy.loginfo(currentMarker)
            rospy.loginfo(timeTaken)
            rospy.loginfo(idList)

def timeSince(timeSinceLast):

    # Get the 'timeThresholdLow' and 'timeThresholdHigh' parameters from the parameter server
    timeThresholdLow = rospy.get_param("~timeThresholdLow", 2)
    timeThresholdHigh = rospy.get_param("~timeThresholdHigh", 6)


    # Init with base value
    plastic = 0
    
    rospy.loginfo(timeSinceLast)

    if currentMarker > 0: 
        
        if timeSinceLast < timeThresholdHigh:
            
            plastic = 1

            rospy.loginfo('decreasing speed')

        elif timeSinceLast > timeThresholdLow:
            
            plastic = 2

            rospy.loginfo('increasing speed')

    

    # Publish 'plastic' as a ROS topic
    plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
    plasticPub.publish(plastic)
    




if __name__ == '__main__':
    makeFolder()
    rosInit()

    rospy.spin()
