#! /usr/bin/env python3
import sys
#sources ros so that packages can be found
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
import rospy
import rospkg

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from points_test import *

def main():
    count=0
    pub = rospy.Publisher('/points', MarkerArray, queue_size=1000)
    pubCorrections = rospy.Publisher('/correctionPoints', MarkerArray, queue_size=1000)
    pubLines = rospy.Publisher('/correctionLines', MarkerArray, queue_size=1000)
    
    dataPointsx, dataPointsy, dataPointsz = generatePoints(0, 10, 0.05)
    CPoints = correctPoints(np.column_stack((dataPointsx, dataPointsy, dataPointsz)), 0.5)
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
    for i in range(len(CPoints)):
        p = Marker()
        p.type = point.SPHERE
        p.header.frame_id = "map"
        p.id = i
        p.action = point.ADD

        p.pose.position.x = CPoints[i][0]
        p.pose.position.y = CPoints[i][1] 
        p.pose.position.z = CPoints[i][2]
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
    correctionLines = MarkerArray()
    for i in range(len(CPoints)-1):
        line = Marker()
        line.type = line.LINE_STRIP
        line.header.frame_id = "map"
        line.id = i
        line.action = line.ADD

        start_point = Point()
        start_point.x = CPoints[i][0]
        start_point.y = CPoints[i][1]
        start_point.z = CPoints[i][2]

        end_point = Point()
        end_point.x = CPoints[i+1][0]
        end_point.y = CPoints[i+1][1]
        end_point.z = CPoints[i+1][2]

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

    while not rospy.is_shutdown():
        print("Publishing points")
        pub.publish(points)
        pubCorrections.publish(correctionPoints)
        pubLines.publish(lines)
        rospy.sleep(1)



if __name__ == '__main__':
    rospy.init_node('rviz_points')
    main()