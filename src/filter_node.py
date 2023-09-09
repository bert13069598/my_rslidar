#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np

class PointCloud2Manager:
    def __init__(self, msg):
        self.msg = msg
        self.x_offset = None
        self.y_offset = None
        self.z_offset = None
        self.i_offset = None

        self.header = msg.header
        self.fields = msg.fields                # x, y, z, intensity
        self.height = msg.height                # num of points : data // point_step // width
        self.width = msg.width                  # 32
        self.is_bigendian = msg.is_bigendian    # boolean
        self.point_step = msg.point_step        # 16
        self.row_step = msg.row_step            # 512
        self.is_dense = msg.is_dense            # boolean
        
        self.sub_data = msg.data                # subscribe data
        self.pub_data = bytearray()             # publish data

    def isOffset(self):
        if self.x_offset is None or \
           self.y_offset is None or \
           self.z_offset is None or \
           self.i_offset is None:
            return False
        else:
            return True

    def findField(self):
        # find field index
        self.x_offset = next((f.offset for f in self.fields if f.name == 'x'), None)         # 0
        self.y_offset  = next((f.offset for f in self.fields if f.name == 'y'), None)        # 1
        self.z_offset = next((f.offset for f in self.fields if f.name == 'z'), None)         # 2
        self.i_offset = next((f.offset for f in self.fields if f.name == 'intensity'), None) # 3

    def decoding(self):
        rows = len(self.sub_data) // self.point_step
        array=np.empty((rows, 4), dtype=np.float32)
        for idx in range(rows):
            array[idx,0] = struct.unpack('f', self.sub_data[idx * self.point_step + self.x_offset:idx * self.point_step + self.x_offset + 4])[0]
            array[idx,1] = struct.unpack('f', self.sub_data[idx * self.point_step + self.y_offset:idx * self.point_step + self.y_offset + 4])[0]
            array[idx,2] = struct.unpack('f', self.sub_data[idx * self.point_step + self.z_offset:idx * self.point_step + self.z_offset + 4])[0]
            array[idx,3] = struct.unpack('f', self.sub_data[idx * self.point_step + self.i_offset:idx * self.point_step + self.i_offset + 4])[0]
        return array
    
    def encoding(self, array):
        for x,y,z,i in array:
            self.pub_data.extend(struct.pack('f', x))
            self.pub_data.extend(struct.pack('f', y))
            self.pub_data.extend(struct.pack('f', z))
            self.pub_data.extend(struct.pack('f', i))
    
    def setPubPC2(self, 
                  header=None,
                  fields=None,
                  height=None,
                  width=None,
                  is_bigendian=None,
                  point_step=None,
                  row_step=None,
                  is_dense=None,
                  data=None):
        pub_msg = PointCloud2()
        pub_msg.header = self.header if header is None else header
        pub_msg.fields = self.fields if fields is None else fields
        pub_msg.height = self.height if height is None else height
        pub_msg.width = self.width if width is None else width
        pub_msg.is_bigendian = self.is_bigendian if is_bigendian is None else is_bigendian
        pub_msg.point_step = self.point_step if point_step is None else point_step
        pub_msg.row_step = self.row_step if row_step is None else row_step
        pub_msg.is_dense = self.is_dense if is_dense is None else is_dense
        pub_msg.data = self.sub_data if data is None else data
        pub.publish(pub_msg)

        


def callback(msg):
    pcd2mng = PointCloud2Manager(msg)
    pcd2mng.findField()

    rospy.loginfo(f"seqence {msg.header.seq}")

    if pcd2mng.isOffset():
        rospy.loginfo(f"points num  {pcd2mng.height}")

        sub_data_np = pcd2mng.decoding()

        sub_data_np[:,2] *= 4.

        pcd2mng.encoding(sub_data_np)

        pcd2mng.setPubPC2(data=pcd2mng.pub_data)
        del pcd2mng
        

rospy.init_node('filter_node')

sub = rospy.Subscriber('rslidar_points', PointCloud2, callback)

pub = rospy.Publisher('filter_points', PointCloud2, queue_size=10)

rospy.spin()