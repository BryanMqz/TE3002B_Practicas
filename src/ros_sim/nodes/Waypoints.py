#! /usr/bin/env python3
import copy
import rospy
from visualization_msgs.msg import Marker, MarkerArray

class Waypoints():
    def __init__(self):
        rospy.init_node('rviz_marker_and_point_tf_publisher')
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 10)
        self.marker = Marker()
        self.marker.header.frame_id = "/world"
        self.marker_arr = MarkerArray()
        self.marker_array = [[1,10],[9,2],[7,10],[2,2],[4,8]]
        self.marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        # Set the scale of the marker
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def place_markers(self):
        for i in range(len(self.marker_array)):
            temp_marker = copy.deepcopy(self.marker)
            # Set the pose of the marker
            temp_marker.id = i
            temp_marker.pose.position.x = self.marker_array[i][0]
            temp_marker.pose.position.y = self.marker_array[i][1]
            temp_marker.pose.position.z = 0.0
            temp_marker.pose.orientation.x = 0.0
            temp_marker.pose.orientation.y = 0.0
            temp_marker.pose.orientation.z = 0.0
            temp_marker.pose.orientation.w = 1.0
            self.marker_arr.markers.append(temp_marker)

    def publishMarkers(self):
        self.marker_pub.publish(self.marker_arr)

if __name__=='__main__':
    while not rospy.is_shutdown():
        w = Waypoints()
        w.place_markers()
        w.publishMarkers
        rospy.rostime.wallsleep(1.0)