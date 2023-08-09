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
timeSinceList = []
minList = []
maxList = []
timeTaken = 0
# currentMarker = 999
buffer = []
start = time.perf_counter()
maxVel = 0
minVel = 0
tick = 0


def checkBuffer(buffer,check):
    buffer.append(check)

    if len(buffer) == 11:
        buffer.pop(0)
    
    tick = 0

    for i in buffer:
        if i == check:
            tick += 1

            if tick == 3:
                return True


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

    # rospy.Subscriber('maxVelocity', Float32, maxVelocityCallback)
    # rospy.Subscriber('minVelocity', Float32, minVelocityCallback)

    
    rospy.on_shutdown(saveCSV)


def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True


# def maxVelocityCallback(msg):

#     global maxVel

#     maxVel = msg.data



# def minVelocityCallback(msg):

#     global minVel

#     minVel = msg.data




def getTag(msg):

    # global idList
    # global currentMarker
    global timeTaken
    # global tick

    for marker in msg.markers:
        
        if checkBuffer(buffer,marker.id):
            if marker.id < 18 and checkDuplicate(idList,marker.id):


                finish = time.perf_counter()
                timeTaken = round(finish-start, 2)


                if len(timeList) > 0:
                    rospy.loginfo('timelist: ' + str(timeList))
                    rospy.loginfo('timelist-1:  ' + str(timeList[-1]))
                    lastTimestamp = timeList[-1]
                    timeSinceLast = round(timeTaken - lastTimestamp, 2)
                    rospy.loginfo('timesincelast: ' + str(timeSinceLast))

                    
                else:
                    rospy.loginfo('timesince: 0')
                    timeSinceLast = 0

                    #timeSinceLast = round(finish - lastTimestamp.get(marker.id, finish), 5)
                    # lastTimestamp[currentMarker] = finish

                    minList.append(minVel)
                    maxList.append(maxVel)
                    timeList.append(timeTaken)
                    idList.append(marker.id)
                    timeSinceList.append(timeSinceLast)
                

                    timeSince(timeSinceLast)

                
                rospy.loginfo(marker.id)
                rospy.loginfo(timeTaken)
                rospy.loginfo(idList)
                

def timeSince(timeSinceLast):

    # Get the 'timeThresholdLow' and 'timeThresholdHigh' parameters from the parameter server
    timeThresholdLow = rospy.get_param("~timeThresholdLow", 2)
    timeThresholdHigh = rospy.get_param("~timeThresholdHigh", 6)


    # Init with base value
    plastic = 0
    
    rospy.loginfo(timeSinceLast)

    if buffer[-1] > 0: 
        
        if timeSinceLast < timeThresholdHigh:
            
            plastic = 1

            rospy.set_param('/max_vel_x', 10)
            rospy.set_param('/acc_lim_x', 1)

            rospy.loginfo('decreasing speed')

        elif timeSinceLast > timeThresholdLow:
            
            plastic = 2
            rospy.set_param('/max_vel_x', 100)
            rospy.set_param('/acc_lim_x', 100)

            rospy.loginfo('increasing speed')

    

    # Publish 'plastic' as a ROS topic
    plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
    plasticPub.publish(plastic)
    




if __name__ == '__main__':
    makeFolder()
    rosInit()

    rospy.spin()
