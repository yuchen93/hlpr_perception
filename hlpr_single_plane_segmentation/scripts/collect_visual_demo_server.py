#!/usr/bin/env python

from hlpr_single_plane_segmentation.srv import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy

markers = {}

def collect_visual_demo(req):
    print "Collecting visual demonstrations"
    print markers[0], markers[1], markers[4]
    response = CollectVisualDemoResponse()
    states = [0,1,2,3]
    actions = [3,3,3,3]
    response.states = states
    response.actions = actions
    return response

def update_marker_pose(data):
    for marker in data.markers:
        curr_pose = marker.pose.pose.position
        if marker.id in markers:
            markers[marker.id].x += curr_pose.x
            markers[marker.id].y += curr_pose.y
            markers[marker.id].z += curr_pose.z
            markers[marker.id].x /= 2
            markers[marker.id].y /= 2
            markers[marker.id].z /= 2
        else:
            markers[marker.id] = curr_pose

def construct_world():
    rospy.sleep(1.0)
    while not (0 in markers and 1 in markers and 4 in markers):
        print "Waiting for markers 0,1 and 4 ..."

    print markers[0], markers[1], markers[4]

def collect_visual_demo_server():
    rospy.init_node('collect_visual_demo_server')
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, update_marker_pose)
    print('Redeay to collect visual demos')
    hs = rospy.Service('collect_visual_demo', CollectVisualDemo, collect_visual_demo)
    construct_world()
    rospy.spin()


if __name__ == "__main__":
    collect_visual_demo_server()
