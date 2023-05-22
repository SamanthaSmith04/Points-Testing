#! /usr/bin/env python3
import sys
import os
#sources ros so that packages can be found
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import numpy as np

from scripts.points_test import *

def main():
    file_path = os.path.dirname(os.path.realpath(__file__)) + "/points_data/"
    rospy.init_node('rviz_points')
    
    pub = rospy.Publisher('/points', MarkerArray, queue_size=1000)
    pubCorrections = rospy.Publisher('/correctionPoints', MarkerArray, queue_size=1000)
    pubLines = rospy.Publisher('/correctionLines', MarkerArray, queue_size=1000)
    dataPointsx = []
    dataPointsy = []
    dataPointsz = []
    c_points = []
    deltas = []
    
    while not rospy.is_shutdown():
        dataPointsx.clear()
        dataPointsy.clear()
        dataPointsz.clear()
        select_values()
        if (inFile == ""):
            dataPointsx, dataPointsy, dataPointsz = generatePoints(minY, maxY, maxInSpacing)
        else:
            #read in data from the input file
            print("Reading in data from " + inFile + "...")
            dataPoints = open(file_path + inFile, "r")
            for line in dataPoints:
                line = line.split()
                dataPointsx.append(float(line[0]))
                dataPointsy.append(float(line[1]))
                dataPointsz.append(float(line[2]))
            dataPoints.close()
        c_points.clear()
        c_points = correctPoints(np.column_stack((dataPointsx, dataPointsy, dataPointsz)), maxOutSpacing)
        deltas.clear()
        deltas = delta(dataPointsx, dataPointsy, dataPointsz, c_points)

        if (outFile != ""): 
            print("Writing data to " + outFile)
            output = open(file_path + outFile, 'w+')
            for i in range(len(c_points)):
                output.write(str(c_points[i][0]) + " " + str(c_points[i][1]) + " " + str(c_points[i][2]) + "\n")
            output.close()
            deltaOutput = outFile.split(".")[0] + "_delta.txt"
            deltaOut = open(file_path + deltaOutput, 'w+')
            for i in range(len(deltas)):
                deltaOut.write(deltas[i].__str__() + "\n")
            deltaOut.close()
        
        points= MarkerArray()
        lines = MarkerArray()
        correctionPoints = MarkerArray()

        ##DATA POINTS
        for i in range(len(dataPointsz)):
            point = Marker()
            
            point.type = point.SPHERE
            point.header.frame_id = "map"
            point.id = i
            point.action = point.ADD

            point.pose.position.x = dataPointsx[i]
            point.pose.position.y = dataPointsy[i]   
            point.pose.position.z = dataPointsz[i]
            point.pose.orientation.x = 0.0
            point.pose.orientation.y = 0.0
            point.pose.orientation.z = 0.0
            point.pose.orientation.w = 1.0
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1

            point.color.r = 0
            point.color.g = 0
            point.color.b = 1.0 * i/len(dataPointsz)
            point.color.a = 1
            
            points.markers.append(point)

        ##CORRECTION POINTS
        for i in range(len(c_points)):
            p = Marker()
            p.type = point.SPHERE
            p.header.frame_id = "map"
            p.id = i
            p.action = point.ADD

            p.pose.position.x = c_points[i][0]
            p.pose.position.y = c_points[i][1] 
            p.pose.position.z = c_points[i][2]
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            p.scale.x = 0.2
            p.scale.y = 0.2
            p.scale.z = 0.2

            p.color.r = 1
            p.color.g = 0
            p.color.b = 0
            p.color.a = 1

            correctionPoints.markers.append(p)

        ##CORRECTION LINES
        for i in range(len(c_points)-1):
            line = Marker()
            line.type = line.LINE_STRIP
            line.header.frame_id = "map"
            line.id = i
            line.action = line.ADD

            start_point = Point()
            start_point.x = c_points[i][0]
            start_point.y = c_points[i][1]
            start_point.z = c_points[i][2]

            end_point = Point()
            end_point.x = c_points[i+1][0]
            end_point.y = c_points[i+1][1]
            end_point.z = c_points[i+1][2]

            line.scale.x = 0.1
            line.scale.y = 0.1
            line.scale.z = 0.1

            line.color.r = 0
            line.color.g = 1.0
            line.color.b = 0
            line.color.a = 1

            line.points.append(start_point)
            line.points.append(end_point)
            lines.markers.append(line)

        print("Publishing points")
        pub.publish(points)
        pubCorrections.publish(correctionPoints)
        pubLines.publish(lines)
        rospy.sleep(1)
        print("continue? (y/n)")
        if (input() != "y" and input() != "Y"):
            exit()


def select_values():
    global outFile
    global inFile
    global minY
    global maxY
    global maxInSpacing
    global maxOutSpacing

    outFile = rospy.get_param('/points_rviz/outputFile')
    inFile = rospy.get_param('/points_rviz/inputFile')

    if (inFile == ""):
        print("Values for test data point genereation (units in meters)")
        print("Enter minimum Y value:")
        minY = float(input())
        print("Enter maximum Y value:")
        maxY = float(input())
        print("Enter maximum input spacing:")
        maxInSpacing = float(input())
    print("Enter maximum output spacing:")
    maxOutSpacing = float(input())
    print("=====================================")
    
    


if __name__ == '__main__':
    main()