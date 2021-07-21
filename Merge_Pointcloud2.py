#! /usr/bin/env python

import rospy
import struct
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class Merge_Pointcloud2:
    def __init__(self):
        rospy.init_node('Merge_Pointcloud2',anonymous=True)
        self.pub1=rospy.Publisher('/merge/pointcloud2',PointCloud2, queue_size=10)
        self.sub1=rospy.Subscriber('/os1_cloud_node/points',PointCloud2,self.recallback1,queue_size=10)
        self.sub2=rospy.Subscriber('/velodyne_points',PointCloud2,self.recallback2,queue_size=10)
        self.point1=None
        rospy.spin()

    def recallback1(self,data):
        pc1=point_cloud2.read_points(data,skip_nans=True, field_names=('x','y','z','intensity'))
        points1=[]
        for p in pc1:
            x=float(p[0])
            y=float(p[1])
            z=float(p[2])
            i=float(p[3])
            pt1=[x,y,z,i]
            points1.append(pt1)
        self.point1=points1

    def recallback2(self,data):
        pc1=point_cloud2.read_points(data,skip_nans=True, field_names=('x','y','z','intensity'))
        points=[]
        for p in pc1:
            x=float(p[0])
            y=float(p[1])
            z=float(p[2])
            i=float(p[3])
            pt1=[x,y,z,i]
            points2.append(pt1)
        points=points+self.point1
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.UINT32, 1),
        ]
        header = Header()
        header.frame_id = "map"
        pc21 = point_cloud2.create_cloud(header, fields, points)
        self.pub1.publish(pc21)

if __name__=='__main__':
    Merge_Pointcloud2()
