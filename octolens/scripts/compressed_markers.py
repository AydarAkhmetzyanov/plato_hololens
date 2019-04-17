#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from octolens.msg import CompressedMarker
from octolens.msg import CompressedMarkerArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

last_array = MarkerArray()

def callback(data):
    global last_array
    last_array=data


def prepare_compressed_marker_array():
    msg = CompressedMarkerArray()
    msg.scale=0.07

    heights_map = {}
    
    global last_array
    for marker in last_array.markers:
        if(marker.action==0):
            msg.scale=marker.scale.z
            for point in marker.points:
                if point.z < 1.9 and point.z>0.1:
                    if (point.x,point.y) in heights_map:
                        if point.z<heights_map[(point.x,point.y)]["min"]:
                            heights_map[(point.x,point.y)]["min"] = point.z
                        if point.z>heights_map[(point.x,point.y)]["max"]:
                            heights_map[(point.x,point.y)]["max"]  = point.z  
                    else:
                        heights_map[(point.x,point.y)] = { "min": point.z, "max": point.z }

    #rospy.loginfo(len(heights_map))

    #for each heightsmap
    for lcoordinates, values in heights_map.iteritems():
        new_marker = CompressedMarker()
        new_marker.x=lcoordinates[0]
        new_marker.y=lcoordinates[1]
        new_marker.z=values["min"]
        new_marker.height=values["max"]-values["min"]+msg.scale
        msg.markers.append(new_marker)

    return msg


def compressed_markers():
    pub = rospy.Publisher('compressed_markers', CompressedMarkerArray, queue_size=1)
    rospy.init_node('compressed_markers', anonymous=True)

    rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, callback, queue_size=1)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        #rospy.loginfo(last_array)
        pub.publish(prepare_compressed_marker_array())
        #rospy.loginfo(prepare_compressed_marker_array())
        rate.sleep()

if __name__ == '__main__':
    try:
        compressed_markers()
    except rospy.ROSInterruptException:
        pass

