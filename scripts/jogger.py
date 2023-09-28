#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import datetime
import rospkg 
import csv
import os
import time


start = time.perf_counter()


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

    dateFormat = "%m%d"
    dateStr = timenow.strftime(dateFormat)

    rp = rospkg.RosPack()
    packagePath = rp.get_path('arLogger')

    logFolder = os.path.join(packagePath, "logs")
    folderName = dateStr


    velPath = os.path.join(logFolder, folderName + "vel")
    posePath = os.path.join(logFolder,folderName + "pose")
    batteryPath = os.path.join(logFolder,folderName + "pose")


    fullpath = os.path.join(path, timenow + "_arlog.csv")

    print (fullpath)

    return path, fullpath, logFolder

def makeFolder():

    path, _ , logFolder= getPath()

    testFile = None

    # test folder permisions and makes log folder
    try:
        testFile = open(os.path.join(path, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(path)

            print ("Log folder created")

        except OSError:
            print("No log folder created")


    testFile.close()
    os.remove(testFile.name)

    testFile = None

    # test folder permisions and makes vel folder
    try:
        testFile = open(os.path.join(logFolder, 'test.txt'), 'w+')
    except IOError:
        try:
            os.mkdir(path)

            print ("Log folder created")

            testFile.close()
            os.remove(testFile.name)

        except OSError:
            print("No log sub-folder created")
    
    testFile.close()
    os.remove(testFile.name)
    


def saveCSV():
    
    _, filename, _ = getPath()

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['ID', 'Time', 'Timesince', 'ROS Time', 'Behave Val', 'Random No', 'State'])
        
        for i in range(len(stateList)):
            writer.writerow([idList[i], timeList[i], timeSinceList[i], rosTimeList[i], beHaveList[i],ranNoList[i],stateList[i]])