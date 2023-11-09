#!/usr/bin/env python3

import rospy
import csv
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospkg
from datetime import datetime
import time
import os
from std_msgs.msg import Int32, String
import random
import threading
import socket

#Global vars
idList = []
idListBuffer = []
idQTY = 0
idQTYList = []


timeList = []
timeSinceList = []
rosTimeList = []
ranNoList = []
activityLevelList = []
activityModeList = []
timeTaken = 0
currentMarker = 999
start = time.perf_counter()
maxVel = 0

timeThreadList =[]
actModeList = []
probableList = []
actLevelList = []




activityLevel = 50
ranDomNo = 50


 # Init with base value
activityMode = 0

timeVar = 0

def rosInit():

    global timeThresholdHigh
    global timeThresholdLow

    rospy.init_node("arLogger")


    ar_subscriber = rospy.Subscriber("ar_pose_marker", AlvarMarkers, getTag)


    # Get the 'timeThresholdLow' and 'timeThresholdHigh' parameters from the parameter server
    timeThresholdLow = rospy.get_param("/timeThresholdLow")
    timeThresholdHigh = rospy.get_param("/timeThresholdHigh")


    # rospy.Subscriber('maxVelocity', Float32, maxVelocityCallback)
    # rospy.Subscriber('minVelocity', Float32, minVelocityCallback)

    rospy.on_shutdown(metricsCSV)
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

    # timeThresholdLo = 'np'
    # timeThresholdHig = timeThresholdLow

    hostName = str(socket.gethostname())

    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    # path = os.path.join(packagePath, "logs")

    path = (packagePath + "/logs/" + hostName + "/TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "/extras/")


    if not os.path.exists(path):
        os.makedirs(path)

    fullpath = os.path.join(path, timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_arlog.csv")
    metricfullpath = os.path.join(path, timenow + hostName + "_TTH_" + str(timeThresholdLow) + "_" + str(timeThresholdHigh) + "_metricslog.csv")

    print (fullpath)

    return path, fullpath, metricfullpath

def makeFolder():

    path, _, _ = getPath()

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
    
    _, filename, _ = getPath()

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['ID', 'Time', 'Timesince', 'ROS Time', 'Activity Level', 'Random No', 'Activity Mode'])
        
        for i in range(len(activityModeList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], rosTimeList[i], activityLevelList[i],ranNoList[i],activityModeList[i]])

def metricsCSV():

    _, _, filename = getPath()

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Activity Mode', 'Probability Value', 'Activity Level', 'Tags Detected'])
        
        for i in range(len(actModeList)):
            writer.writerow([timeThreadList[i], actModeList[i], probableList[i], actLevelList[i], idQTY[i]])





def checkDuplicate(iterable,check):
    for i in iterable:
        if i == check:
            return True



def getTag(msg):

    global idList
    global idListBuffer
    global currentMarker
    global timeTaken
    global activityModeList
    global ranDomNo
    global activityLevel
    global idQTY


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

                idQTY += 1

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

    #This function will increase activity level by 5 if it sees multiple AR Tags in a span of less than timethresholdlow

    global timeThresholdLow  
    global activityLevel
    global timeVar
   
    rospy.loginfo('Time since last: %s', str(timeSinceLast))


    if currentMarker > 0: 
        
        if timeSinceLast <= timeThresholdLow:

            activityLevel += 5

            timeVar = 0
 

         # Ensure activityLevel within a reasonable range (e.g., between 0 and 100)
        activityLevel = max(0, min(100, activityLevel))

        activityModeList.append(activityMode)

        _, rosTimeNow = getTime()


        rosTimeList.append(rosTimeNow)



def checkTimeHigh():

    #This function will count to timethresholdhigh in seconds if an AR Tag is not detected then decrease activity level by 5

    global timeThresholdHigh
    global timeVar
    global activityLevel

    while not rospy.is_shutdown():
        time.sleep(1)  # Sleep for 1 second
        timeVar += 1

        if timeVar >= timeThresholdHigh:
            activityLevel -= 5

            activityLevel = max(0, min(100, activityLevel))

            timeVar = 0


       

def beHaveFun():

    #This function will generate a random number to determine activity mode based on probability

    global activityMode
    global activityLevel
    global ranDomNo
    global start
    global actModeList
    global timeThreadList
    global probableList
    global actLevelList
    global idQTY
    global idQTYList


    while not rospy.is_shutdown():
        
        ranDomNo = random.randrange(0,101)

        # rospy.loginfo('behave: ' + str(activityLevel) + ' random: ' + str(ranDomNo))            
        # 
        finish = time.perf_counter()
   
        timeTaken = round(finish-start, 2)

        


        if activityLevel < ranDomNo:

            activityMode = 0

            activityOutput = 'Patrol - (Neophobic)'

            # rospy.loginfo('Patrol Activity Mode - (Neophobic)')


        else:
            
            activityMode = 1

            activityOutput = 'Explore - (Neophilic)'
        
            # rospy.loginfo('Explore Activity Mode - (Neophilic)')

        
        actModeList.append(activityMode)
        timeThreadList.append(timeTaken)
        probableList.append(ranDomNo)
        actLevelList.append(activityLevel)
        idQTYList.append(idQTY)



        probabilityOutput = 'Activity Level: ' + str(activityLevel) + ' Random: ' + str(ranDomNo) 
        probabilityMessage = String()
        probabilityMessage.data = probabilityOutput

        # Publish 'plastic' as a ROS topic
        activityPub = rospy.Publisher('activityTopic', String, queue_size=10)
        activityPub.publish(probabilityMessage)
        
        activityPubOutput = 'Activity Mode: ' + str(activityOutput)
        activityMessage = String()
        activityMessage.data = activityPubOutput

        activityPub.publish(activityPubOutput)

        metricsOutput = metricsOutput = "{},{},{},{},{}".format(timeTaken, activityMode, ranDomNo, activityLevel, idQTY)
        metricsMessage = String()
        metricsMessage.data = metricsOutput

        metricsPub = rospy.Publisher('metricsTopic', String, queue_size=10)
        metricsPub.publish(metricsMessage)        


        # Publish 'plastic' as a ROS topic
        plasticPub = rospy.Publisher('plasticTopic', Int32, queue_size=10)
        plasticPub.publish(activityMode)

        time.sleep(1)
    
    



if __name__ == '__main__':
    rosInit()
    # makeFolder()


    plastic_thread = threading.Thread(target=beHaveFun)
    plastic_thread.daemon = True  # This makes the thread exit when the main program exits
    plastic_thread.start()  # Start the thread

    plastic_thread = threading.Thread(target=checkTimeHigh)
    plastic_thread.daemon = True  # This makes the thread exit when the main program exits
    plastic_thread.start()  # Start the thread


    
    rospy.spin()