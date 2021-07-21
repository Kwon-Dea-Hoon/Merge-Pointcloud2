#! /usr/bin/env python

import rospy
import struct
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class Merge_Pointcloud2:
    def __init__(self):
        rospy.init_node('newdata',anonymous=True)
        self.pub1=rospy.Publisher('/merge/pointcloud2',PointCloud2, queue_size=10)
        self.sub1=rospy.Subscriber('/os1_cloud_node/points',PointCloud2,self.recallback1,queue_size=10)
        self.sub2=rospy.Subscriber('/velodyne_points',PointCloud2,self.recallback2,queue_size=10)
        self.point1=None
        rospy.spin()

    def recallback1(self,data):
        pc1=point_cloud2.read_points(data,skip_nans=True, field_names=('x','y','z','intensity'))
        global points1
        points1=[]
        for p1 in pc1:
            x1=float(p1[0])
            y1=float(p1[1])
            z1=float(p1[2])
            in1=float(p1[3])
            pt1=[x1,y1,z1,in1]
            points1.append(pt1)
        self.point1=points1

    def recallback2(self,data):
        pc1=point_cloud2.read_points(data,skip_nans=True, field_names=('x','y','z','intensity'))
        global points2
        points2=[]
        for p1 in pc1:
            x1=float(p1[0])
            y1=float(p1[1])
            z1=float(p1[2])
            in1=float(p1[3])
            pt1=[x1,y1,z1,in1]
            points2.append(pt1)
        points2=points2+self.point1
        fields1 = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.UINT32, 1),
        ]
        header1 = Header()
        header1.frame_id = "map"
        pc21 = point_cloud2.create_cloud(header1, fields1, points2)
        self.pub1.publish(pc21)

if __name__=='__main__':
    Merge_Pointcloud2()
