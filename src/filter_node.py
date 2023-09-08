#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import struct

def decoding(data, i, offset, axis_offset):
    return struct.unpack('f', data[i * offset + axis_offset:i * offset + axis_offset + 4])[0]

def encoding(data, axis):
    data.extend(struct.pack('f', axis))
    return data

def callback(msg):
    # get field information from pcd msg
    fields = msg.fields

    # find field index
    x_offset = next((f.offset for f in fields if f.name == 'x'), None)         # 0
    y_offset = next((f.offset for f in fields if f.name == 'y'), None)         # 1
    z_offset = next((f.offset for f in fields if f.name == 'z'), None)         # 2
    i_offset = next((f.offset for f in fields if f.name == 'intensity'), None) # 3

    rospy.loginfo(f"seqence {msg.header.seq}")

    if x_offset is not None and y_offset is not None and z_offset is not None and i_offset is not None:
        # get x, y, z, intensity from data array
        data = msg.data
        offset = len(fields) * 4
        points_num = len(data) // 32 // offset

        rospy.loginfo(f"points num  {points_num}")

        modified_data = bytearray()

        for i in range(len(data)//offset):
            x = decoding(data, i, offset, x_offset)
            y = decoding(data, i, offset, y_offset)
            z = decoding(data, i, offset, z_offset)
            intensity = decoding(data, i, offset, i_offset)

            # print(x,y,z,intensity)

            z *= 4

            modified_data = encoding(modified_data, x)
            modified_data = encoding(modified_data, y)
            modified_data = encoding(modified_data, z)
            modified_data = encoding(modified_data, intensity)
            
        # create new PointCloud2 data based on modified data
        pub_msg = PointCloud2()
        pub_msg.header = msg.header
        pub_msg.fields = msg.fields
        pub_msg.height = msg.height
        pub_msg.width = msg.width
        pub_msg.is_bigendian = msg.is_bigendian
        pub_msg.point_step = msg.point_step
        pub_msg.row_step = msg.row_step
        pub_msg.is_dense = msg.is_dense
        pub_msg.data = modified_data
        # pub_msg.data = msg.data

        # publish 'filter_points' topic
        pub.publish(pub_msg)
        # if points_num % 5 == 0:
            # pub.publish(pub_msg)


rospy.init_node('filter_node')

sub = rospy.Subscriber('rslidar_points', PointCloud2, callback)

pub = rospy.Publisher('filter_points', PointCloud2, queue_size=10)

rospy.spin()