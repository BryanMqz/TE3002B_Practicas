#! /usr/bin/env python3
import copy
import rospy
from visualization_msgs.msg import Marker, MarkerArray

class Waypoints():
    def __init__(self):
        rospy.init_node('rviz_markers')
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 10)
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker_arr = MarkerArray()
        self.marker.header.stamp = rospy.Time.now()
        self.marker_array = [[5,8],[9,2],[1,5],[7,10],[4,1]]
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        # Set the scale of the marker
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        # Set the color
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

    def place_markers(self):
        for i in range(len(self.marker_array)):
            tmpmrk = copy.deepcopy(self.marker)
            # Set the pose of the marker
            tmpmrk.id = i
            tmpmrk.pose.position.x = self.marker_array[i][0]-5.4
            tmpmrk.pose.position.y = self.marker_array[i][1]-5.5
            tmpmrk.pose.position.z = 0.0
            tmpmrk.pose.orientation.x = 0.0
            tmpmrk.pose.orientation.y = 0.0
            tmpmrk.pose.orientation.z = 0.0
            tmpmrk.pose.orientation.w = 1.0
            self.marker_arr.markers.append(tmpmrk)

    def publishMarkers(self):
        self.marker_pub.publish(self.marker_arr)

if __name__=='__main__':
    while not rospy.is_shutdown():
        w = Waypoints()
        w.place_markers()
        w.publishMarkers()
        rospy.rostime.wallsleep(1.0)