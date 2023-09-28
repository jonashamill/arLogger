#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import datetime
import rospkg 
import csv
import os
import time


startTime = time.perf_counter()


def getTime():

    #grabbing time and date to provide unique ID for logs
    dateTime = datetime.now()
    dtString = dateTime.strftime("%Y%m%d%H%M%S") #ISO 8601 Standard

    rosTimeUnf = rospy.Time.now()

    rosCurrentTime = datetime.fromtimestamp(rosTimeUnf.to_sec())

    rosTime = rosCurrentTime.strftime("%H:%M: %S")

    return dtString, rosTime


def getPath():

    timenow, _ = getTime()

    dateFormat = "%m%d"
    dateStr = timenow.strftime(dateFormat)

    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    logFolder = os.path.join(packagePath, "logs")
    folderName = dateStr


    velPath = os.path.join(logFolder, folderName + "vel")
    posePath = os.path.join(logFolder,folderName + "pose")
    batPath = os.path.join(logFolder,folderName + "pose")


    velFullpath = os.path.join(velPath, timenow + "_vellog.csv")
    poseFullpath = os.path.join(posePath, timenow + "_poselog.csv")
    batFullpath = os.path.join(batPath, timenow + "_batlog.csv")

    print (velFullpath)
    print(poseFullpath)
    print(batFullpath)

    return logFolder, velPath, velFullpath, posePath, poseFullpath, batPath, batFullpath

def makeFolder():

    logFolder, velPath, _, posePath, _, batPath, _= getPath()

    testFile = None

    # test folder permisions and makes log folder
    try:
        testFile = open(os.path.join(logFolder, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(logFolder)

            print ("Log folder created")

        except OSError:
            print("No log folder created")


    testFile.close()
    os.remove(testFile.name)


    # test folder permisions and makes vel folder
    try:
        testFile = open(os.path.join(velPath, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(velPath)

            print ("Vel folder created")

            testFile.close()
            os.remove(testFile.name)

        except OSError:
            print("No vel sub-folder created")
    
    testFile.close()
    os.remove(testFile.name)
    

    # test folder permisions and makes vel folder
    try:
        testFile = open(os.path.join(posePath, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(posePath)

            print ("Vel folder created")

            testFile.close()
            os.remove(testFile.name)

        except OSError:
            print("No vel sub-folder created")
    
    testFile.close()
    os.remove(testFile.name)

    # test folder permisions and makes vel folder
    try:
        testFile = open(os.path.join(batPath, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(batPath)

            print ("Vel folder created")

            testFile.close()
            os.remove(testFile.name)

        except OSError:
            print("No vel sub-folder created")
    
    testFile.close()
    os.remove(testFile.name)



def velCallback(msg):

    global startTime

    finishTime = time.perf_counter()

    timeStamp = round(finishTime-startTime, 2)


    _, velPath, _, _, _, _, _ = getPath()


    lin_x = msg.linear.x
    ang_z = msg.angular.x
    _, timeNow = getTime()
    
    velData = [timeNow, timeStamp, lin_x, ang_z]

    velHeaders =" 'Ros Time', 'Time Stamp', 'Linear Velocity', 'Angular Velocity' "

    saveCSV(velPath, velData, velHeaders)
    


def saveCSV(filename, data, headers):

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([headers])
        
        writer.writerow([data])