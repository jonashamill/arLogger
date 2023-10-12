#!/usr/bin/env python3

import rospy
import csv
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
from datetime import datetime
import time
import os
from std_msgs.msg import Int32
import random
import threading

#Global vars
idList = []
idListBuffer = []


timeList = []
timeSinceList = []
rosTimeList = []
ranNoList = []
activityLevelList = []
stateList = []
timeTaken = 0
currentMarker = 999
start = time.perf_counter()
maxVel = 0


activityLevel = 50
ranDomNo = 50


 # Init with base value
plastic = 0

timeVar = 0

def rosInit():

    global timeThresholdHigh
    global timeThresholdLow

    rospy.init_node("arLogger")


    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, getTag)


    # Get the 'timeThresholdLow' and 'timeThresholdHigh' parameters from the parameter server
    timeThresholdLow = rospy.get_param("~timeThresholdLow", 2)
    timeThresholdHigh = rospy.get_param("~timeThresholdHigh", 6)


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

    fullpath = os.path.join(path, timenow + "_arlog.csv")

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
        writer.writerow(['ID', 'Time', 'Timesince', 'ROS Time', 'Activity Level', 'Random No', 'State'])
        
        for i in range(len(stateList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], rosTimeList[i], activityLevelList[i],ranNoList[i],stateList[i]])





def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True



def getTag(msg):

    global idList
    global idListBuffer
    global currentMarker
    global timeTaken
    global stateList
    global ranDomNo
    global activityLevel


    for marker in msg.markers:
        

        if marker.id != currentMarker:
            
            finish = time.perf_counter()

            
            timeTaken = round(finish-start, 2)
            currentMarker = marker.id
            
            
            

            if checkDuplicate(idListBuffer, currentMarker) or currentMarker > 19:
            
                if 19 < currentMarker < 26:

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


                rosTimeList.append(rosTimeNow)
                activityLevelList.append(activityLevel)
                ranNoList.append(ranDomNo)
                timeList.append(timeTaken)
                
                idListBuffer.append(currentMarker)

                if len(idListBuffer) > 5: #this will be replaced with time
                    idList.extend(idListBuffer)
                    idListBuffer =[]
                
                
                timeSinceList.append(timeSinceLast)
                
            

                checktimeLow(timeSinceLast)


            rospy.loginfo("ID: " + str(currentMarker))
            rospy.loginfo("Time Taken: " + str(timeTaken))
            rospy.loginfo("id List: " + str(idList))
            rospy.loginfo("id Buffer: " + str(idListBuffer))
            

def checktimeLow(timeSinceLast):

    global timeThresholdLow  

    global activityLevel

    global timeVar
   
    rospy.loginfo('Time since last: %s', str(timeSinceLast))


    if currentMarker > 0: 
        
        if timeSinceLast <= timeThresholdLow:

            activityLevel += 5

            timeVar = 0
 

         # Ensure activityLevel within a reasonable range (e.g., between 0 and 100)
        activityLevel = max(1, min(100, activityLevel))

        stateList.append(plastic)

        _, rosTimeNow = getTime()


        rosTimeList.append(rosTimeNow)



def checkTimeHigh():
    global timeThresholdHigh
    global timeVar
    global activityLevel

    while not rospy.is_shutdown():
        time.sleep(1)  # Sleep for 1 second
        timeVar += 1

        if timeVar >= timeThresholdHigh:
            with activityLevel:
                activityLevel -= 5

                activityLevel = max(1, min(100, activityLevel))

            timeVar = 0


       

def beHaveFun():

    global plastic
    global activityLevel
    global ranDomNo

    

    while not rospy.is_shutdown():
        
        ranDomNo = random.randrange(1,101)

        rospy.loginfo('behave: ' + str(activityLevel) + ' random: ' + str(ranDomNo))


        if activityLevel < ranDomNo:

            plastic = 1


            rospy.loginfo('Patrol Mode - (Neophobic)')


        else:
            
            plastic = 2
        
            rospy.loginfo('Explore Mode - (Neophilic)')



        # Publish 'plastic' as a ROS topic
        plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
        plasticPub.publish(plastic)

        time.sleep(1)

if __name__ == '__main__':
    rosInit()
    makeFolder()

    activityLevelLock = threading.Lock()


    plastic_thread = threading.Thread(target=beHaveFun)
    plastic_thread.daemon = True  # This makes the thread exit when the main program exits
    plastic_thread.start()  # Start the thread

    plastic_thread = threading.Thread(target=checkTimeHigh)
    plastic_thread.daemon = True  # This makes the thread exit when the main program exits
    plastic_thread.start()  # Start the thread


    
    rospy.spin()