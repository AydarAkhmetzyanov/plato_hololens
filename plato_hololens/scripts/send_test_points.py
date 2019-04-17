#!/usr/bin/env python
import rospy
from plato_hololens.msg import DenseHumans, DenseHuman, MyObjects, MyObject
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import base64

def talker():
    pub_hum = rospy.Publisher('humans_poses_publisher', DenseHumans, queue_size=1)
    pub_obj = rospy.Publisher('objects_publisher', MyObjects, queue_size=1)
    rospy.init_node('pose_talker', anonymous=True)
    classes = ['cat','dog','car','person','table','chair','window','door','tv','keyboard','bottle']
    r = rospy.Rate(1)  # 10hz

    while not rospy.is_shutdown():
        msg_hum = DenseHumans()
        msg_obj = MyObjects()

        # Create objects
        for k in range(10):
            obj = MyObject()
            obj.class_id = k
            point = Point()
            point.x = (k*3)-10
            point.y = 0
            point.z = 0
            obj.pose = point
            msg_obj.objects.append(obj)
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_obj.header = header
        #rospy.loginfo(msg_obj)
        pub_obj.publish(msg_obj)

        #humans
        with open("humanDense.png", "rb") as image_file: 
            encoded_file = base64.b64encode(image_file.read())

        human = DenseHuman()
        point = Point()
        point.x=5
        point.y = 1
        point.z=0
        human.pose = point
        #file
        human.image_base64 = encoded_file
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_hum.header = header
        msg_hum.people.append(human)

        human = DenseHuman()
        point = Point()
        point.x=0
        point.y = 1
        point.z=0
        human.pose = point
        #file
        human.image_base64 = encoded_file
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_hum.header = header
        msg_hum.people.append(human)


        rospy.loginfo(msg_hum)
        pub_hum.publish(msg_hum)

        
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
