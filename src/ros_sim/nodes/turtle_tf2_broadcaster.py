#!/usr/bin/env python3 
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import turtlesim.msg
from nav_msgs.msg import Path

path = Path()

def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    pose = PoseStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x - 5.5
    t.transform.translation.y = msg.y - 5.5
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

    path.header = t.header
    pose.header = t.header
    pose.pose.position.x = t.transform.translation.x
    pose.pose.position.y = t.transform.translation.y
    pose.pose.position.z = t.transform.translation.z
    pose.pose.orientation = t.transform.rotation
    path.poses.append(pose)
    path_pub.publish(path)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    path_pub = rospy.Publisher('/%s/path' % turtlename, Path, queue_size=10)
    
    rospy.spin()