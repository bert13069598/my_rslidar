#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import struct

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
            x = struct.unpack('f', data[i * offset + x_offset:i * offset + x_offset + 4])[0]
            y = struct.unpack('f', data[i * offset + y_offset:i * offset + y_offset + 4])[0]
            z = struct.unpack('f', data[i * offset + z_offset:i * offset + z_offset + 4])[0]
            intensity = struct.unpack('f', data[i * offset + i_offset:i * offset + i_offset + 4])[0]

            # print(x,y,z,intensity)

            z *= 10

            modified_data.extend(struct.pack('f', x))
            modified_data.extend(struct.pack('f', y))
            modified_data.extend(struct.pack('f', z))
            modified_data.extend(struct.pack('f', intensity))
            
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
        pub_msg.data = bytes(modified_data)
        # pub_msg.data = msg.data

        # publish 'filter_points' topic
        pub.publish(pub_msg)


rospy.init_node('filter_node')

sub = rospy.Subscriber('rslidar_points', PointCloud2, callback)

pub = rospy.Publisher('filter_points', PointCloud2, queue_size=10)

rospy.spin()