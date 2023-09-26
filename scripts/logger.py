#!/usr/bin/env python3

import rospy
import csv
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
from datetime import datetime
import time
import os
from std_msgs.msg import Int32, Float32
import random
import threading

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


beHave = 50


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
        writer.writerow(['ID', 'Time', 'Timesince', 'ROS Time', 'beHave Val', 'State'])
        
        for i in range(len(stateList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], minList[i], maxList[i],stateList[i]])





def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True



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
            
            
            

            if checkDuplicate(idList, currentMarker) or currentMarker > 19:
            
                if 19 < currentMarker < 25:

                    rospy.loginfo('I SEE A ROBOT WITH ID: ' + str(currentMarker) )

                else:
                
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


            rospy.loginfo("ID: " + str(currentMarker))
            rospy.loginfo("Time Taken: " + str(timeTaken))
            rospy.loginfo("id List: " + str(idList))
            

def timeSince(timeSinceLast):

    # Get the 'timeThresholdLow' and 'timeThresholdHigh' parameters from the parameter server
    timeThresholdLow = rospy.get_param("~timeThresholdLow", 2)
    timeThresholdHigh = rospy.get_param("~timeThresholdHigh", 6)


    global beHave
   
    rospy.loginfo('Time since last: %s', str(timeSinceLast))


    if currentMarker > 0: 
        
        if timeSinceLast < timeThresholdHigh:

            beHave += 1


        elif timeSinceLast > timeThresholdLow:


            beHave -= 1
 

         # Ensure beHave stays within a reasonable range (e.g., between 0 and 100)
        beHave = max(1, min(100, beHave))

        stateList.append(plastic)

        _, rosTimeNow = getTime()


        minList.append(rosTimeNow)
            
       
    

def beHaveFun():

    global plastic
    global beHave

    

    while not rospy.is_shutdown():
        
        ranDomNo = random.randrange(1,101)

        if beHave > ranDomNo:

            plastic = 1

            rospy.loginfo('Patrol Mode')


        else:
            
            plastic = 2
        
            rospy.loginfo('Explore Mode')



    # Publish 'plastic' as a ROS topic
    plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
    plasticPub.publish(plastic)

if __name__ == '__main__':
    rosInit()
    makeFolder()

    plastic_thread = threading.Thread(target=beHaveFun)
    plastic_thread.daemon = True  # This makes the thread exit when the main program exits
    plastic_thread.start()  # Start the thread
    
    rospy.spin()
