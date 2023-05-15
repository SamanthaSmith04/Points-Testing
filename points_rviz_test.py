import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node('rviz_points')
    pub = rospy.Publisher('points', Marker, queue_size=10)
    
    point = Marker()
    point.type = Marker.POINTS
    point.id = "marker1"

    point.pose.position.x = 0
    point.pose.position.y = 0
    point.pose.position.z = 0
    point.pose.orientation.x = 0.0
    point.pose.orientation.y = 0.0
    point.pose.orientation.z = 0.0
    point.pose.orientation.w = 1.0

    while not rospy.is_shutdown():
        pub.publish(point)
        rospy.sleep(1)



if __name__ == '__main__':
    main()