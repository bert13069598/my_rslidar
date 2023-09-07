#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import pcl

def callback(msg):
    # 포인트 클라우드 메시지에서 필드 정보를 가져옵니다.
    fields = msg.fields

    # 필요한 필드 인덱스를 찾습니다 (예: 'x', 'y', 'z' 필드를 가져옵니다).
    x_idx = next((i for i, f in enumerate(fields) if f.name == 'x'), None)
    y_idx = next((i for i, f in enumerate(fields) if f.name == 'y'), None)
    z_idx = next((i for i, f in enumerate(fields) if f.name == 'z'), None)

    
    rospy.loginfo(f"Point size {msg.header.seq}")


    if x_idx is not None and y_idx is not None and z_idx is not None:
        # 데이터 배열에서 x, y, z 값을 가져옵니다.
        data = msg.data
        point_size = len(fields)
        point_count = len(data) // point_size
        
        rospy.loginfo(f"Point size {point_count}")

        for i in range(point_count):
            x = data[i * point_size + x_idx]
            y = data[i * point_size + y_idx]
            z = data[i * point_size + z_idx]

            # rospy.loginfo(f"Point {i+1}: x={x}, y={y}, z={z}")


rospy.init_node('filter_node')

sub = rospy.Subscriber('rslidar_points', PointCloud2, callback)

pub = rospy.Publisher('filter_points', PointCloud2, queue_size=10)

rospy.spin()