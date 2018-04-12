#!/usr/bin/env python

from hlpr_single_plane_segmentation.srv import *
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose

GRID_WORLD_WIDTH = 5
GRID_WORLD_HEIGHT = 5

x_side = 0.1
y_side = 0.1

markers = {}

tfBuffer = tf2_ros.Buffer()       
listener=tf2_ros.TransformListener(tfBuffer)

def collect_visual_demo(req):
    rospy.loginfo("Collecting visual demonstrations")
    printed = False
    while not (7 in markers):
        if not printed:
            rospy.loginfo("Waiting for marker 7 ...")
            printed = True
    states = []
    actions = []
    option = int(input('Enter 1 to record demo, 0 to end:'))
    while option != 0:
        transform = tfBuffer.lookup_transform('ar_marker_0','ar_marker_7',rospy.Time(0), rospy.Duration(1.0))
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = 'ar_marker_7'
        pt = tf2_geometry_msgs.do_transform_pose(ps, transform)
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = 'ar_marker_0'
        x = pt.pose.position.x
        y = pt.pose.position.y
        curr_state = get_state(x, y)
        states.append(curr_state)
        rospy.loginfo('state recorded: '+str(curr_state))
        option = int(input('Enter 1 to record demo, 0 to end:'))
         
    response = CollectVisualDemoResponse()
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

def get_state(x,y):
    global x_side, y_side, num_states
    state = num_states - 1 - (math.floor(x/x_side)*GRID_WORLD_WIDTH + math.floor(y/y_side))
    return state

def distance(point1, point2):
    return math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2+(point1.z-point2.z)**2)

def construct_world():
    #             marker1
    #
    #
    #
    #marker4      marker0
    global frames_client, x_side, y_side, num_states
    rospy.sleep(1.0)
    printed = False
    while not (0 in markers and 1 in markers and 4 in markers):
        if not printed:
            rospy.loginfo("Waiting for markers 0,1 and 4 ...")
            printed = True
    rospy.loginfo("Constructing working environment ...")
    width = distance(markers[0],markers[4])
    height = distance(markers[0], markers[1])
    rospy.loginfo("workspace size:"+str(width)+" " +str(height))
    x_side = height / (GRID_WORLD_HEIGHT - 1)
    y_side = width / (GRID_WORLD_WIDTH - 1)
    rospy.loginfo("cell size:"+str(x_side)+" "+str(y_side))
    num_states = GRID_WORLD_WIDTH*GRID_WORLD_HEIGHT
    parent_frames = ["ar_marker_0"]*num_states
    tf_frames_request = BroadcastObjectFramesRequest()
    tf_frames_request.parent_frames = parent_frames
    child_frames = []
    for frame in range(num_states):
        child_frames.append("state_"+str(frame))
    tf_frames_request.child_frames = child_frames
    poses = []
    for frame in range(num_states):
        pose = Pose()
        pose.position.x = x_side * ((num_states - 1 - frame) // GRID_WORLD_WIDTH)
        pose.position.y = y_side * ((num_states - 1 - frame) % GRID_WORLD_WIDTH)
        pose.orientation.w = 1
        poses.append(pose)
    tf_frames_request.poses = poses
    tf_res = frames_client(tf_frames_request)
    if tf_res is False:
        rospy.logerr("Update failed")
        return False
    rospy.loginfo('Detecting Features ...')
    features = {} # [is_goal[marker8], is_init[purple], is_black[black]]
    printed = False
    while not (8 in markers):
        if not printed:
            rospy.loginfo("Waiting for marker 8 ...")
            printed = True
    black_transform = tfBuffer.lookup_transform('ar_marker_0','black',rospy.Time(0), rospy.Duration(1.0))
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = 'black'
    pt = tf2_geometry_msgs.do_transform_pose(ps, black_transform)
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = 'ar_marker_0'
    black_x = pt.pose.position.x
    black_y = pt.pose.position.y
    black_state = get_state(black_x,black_y)

    rospy.loginfo('Balck center state: '+str(black_state))
    rospy.loginfo('Redeay to collect visual demos')
    return True

def collect_visual_demo_server():
    global frames_client
    rospy.init_node('collect_visual_demo_server')
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, update_marker_pose)
    rospy.wait_for_service('broadcast_object_frames')
    frames_client = rospy.ServiceProxy("broadcast_object_frames", BroadcastObjectFrames)
    rospy.loginfo('Redeay to collect visual demos')
    hs = rospy.Service('collect_visual_demo', CollectVisualDemo, collect_visual_demo)
    if not construct_world():
        rospy.logerr("Failed to construct environment")
        return 
    rospy.spin()


if __name__ == "__main__":
    collect_visual_demo_server()
