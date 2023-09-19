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
stateList = []
timeTaken = 0
currentMarker = 999
start = time.perf_counter()
maxVel = 0

 # Init with base value
plastic = 0

def rosInit():

    rospy.init_node("arLogger")


    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, getTag)

    # rospy.Subscriber('maxVelocity', Float32, maxVelocityCallback)
    # rospy.Subscriber('minVelocity', Float32, minVelocityCallback)

    
    rospy.on_shutdown(saveCSV)

def getTime():

    #grabbing time and date to provide unique ID for logs
    dateTime = datetime.now()
    dtString = dateTime.strftime("%Y%m%d%H%M%S") #ISO 8601 Standard

    rosTimeUnf = rospy.Time.now()

    rosCorrentTime = datetime.fromtimestamp(rosTimeUnf.to_sec())

    rosTime = rosCorrentTime.strftime("%H:%M: %S")

    return dtString, rosTime


def getPath():

    timenow, _ = getTime()

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
        writer.writerow(['ID', 'Time', 'Timesince', 'ROS Time', 'Max Velocity', 'State'])
        
        for i in range(len(stateList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], minList[i], maxList[i],stateList[i]])





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

    global idList
    global currentMarker
    global timeTaken
    global stateList

    for marker in msg.markers:
        

        if marker.id != currentMarker:
            
            finish = time.perf_counter()

            
            timeTaken = round(finish-start, 2)
            currentMarker = marker.id
            
            
            

            if checkDuplicate(idList, currentMarker) == True:
                continue
            else:

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

                # Publish 'tag' as a ROS topic
                tagPub = rospy.Publisher('tagTopic', Int32, queue_size=10)
                tagPub.publish(True)

                _, rosTimeNow = getTime()


                minList.append(rosTimeNow)
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


   

    # rospy.set_param('/max_vel_x', 0.25)
    # rospy.set_param('/min_vel_x', 0.1)
    # rospy.set_param('/acc_lim_x', 1.0)
    
    rospy.loginfo('Time since last: %s', str(timeSinceLast))

    plastic = 0

    if currentMarker > 0: 
        
        if timeSinceLast < timeThresholdHigh:
            
            plastic = 1

            # rospy.set_param('/max_vel_x', 0.1)
            # rospy.set_param('/min_vel_x', 0.1)
            # rospy.set_param('/acc_lim_x', 1.0)


            rospy.loginfo('Patrol Mode')

        elif timeSinceLast > timeThresholdLow:
            
            plastic = 2
            # rospy.set_param('/max_vel_x', 0.4)
            # rospy.set_param('/min_vel_x', 0.3)
            # rospy.set_param('/acc_lim_x', 1.0)



            rospy.loginfo('Explore Mode')

   
        stateList.append(plastic)

        _, rosTimeNow = getTime()


        minList.append(rosTimeNow)


        # Publish 'plastic' as a ROS topic
        plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
        plasticPub.publish(plastic)
    





if __name__ == '__main__':
    rosInit()
    makeFolder()
    

    rospy.spin()
