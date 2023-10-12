#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

prev_means = np.empty((0, 4), dtype=np.float32)

class AveragePooling:
    def __init__(self, array):
        self.channel = 32
        self.kernel = 2
        self.threshold = 0.2
        self.ap_r = None

        self.array = array[np.newaxis, :, :]
        self.array = np.transpose(self.array, (2, 0, 1)).reshape(4, -1, self.channel)
        self.means = self.get_means()

    def get_means(self):
        # 1D: x,y,z,i
        # 2D: horizontal
        # 3D: channel(vertical)
        _, col, _ = self.array.shape

        # average pooling
        kernel = 2
        ap_c, self.ap_r = col // kernel, col % kernel
        # print(f'{col} % 2 = {ap_c} + {self.ap_r}')
        ap_idx = np.arange(0, kernel * ap_c).reshape(ap_c, kernel).T
        return np.nanmean(self.array[:, ap_idx, :], axis=1)

    def sync_means(self):
        global prev_means

        _, r1, _ = self.means.shape
        _, r2, _ = prev_means.shape
        if r1 > r2:
            nan_arr = np.full_like(np.empty((4, 1, 32), dtype=np.float32), np.nan)
            nan_arr = np.repeat(nan_arr, r1 - r2, axis=1)
            prev_means = np.concatenate((prev_means, nan_arr), axis=1)
        elif r1 < r2:
            prev_means = prev_means[:, :r1, :]
        _, r1, _ = self.means.shape
        _, r2, _ = prev_means.shape
        

    def filtering(self):
        global prev_means

        # d = x**2 + y**2 + z**2
        distance = np.sum((self.means[:3, :, :] - prev_means[:3, :, :]) ** 2, axis=0)

        prev_means = self.means

        # filtering
        mask = np.repeat(self.threshold < distance, self.kernel, axis=0)[np.newaxis, :]
        nan_arr = np.full_like(np.empty((1, self.ap_r, 32), dtype=np.float32), np.nan)
        mask = np.concatenate((mask, nan_arr), axis=1)
        mask = np.tile(mask, (4, 1, 1))
        filtered_array = np.where(mask, self.array, np.nan)
        return np.transpose(filtered_array, (1, 2, 0)).reshape(-1, 4)


def dynamic_detection(array):
    global prev_means

    ap = AveragePooling(array)

    if len(prev_means) == 0:
        prev_means = ap.means
        return np.full_like(array, np.nan)

    ap.sync_means()
    return ap.filtering()


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
        self.y_offset = next((f.offset for f in self.fields if f.name == 'y'), None)        # 4
        self.z_offset = next((f.offset for f in self.fields if f.name == 'z'), None)         # 8
        self.i_offset = next((f.offset for f in self.fields if f.name == 'intensity'), None) # 12

    def decoding(self):
        num_points = len(self.sub_data) // self.point_step
        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4')])
        array = np.frombuffer(self.sub_data, dtype=dtype, count=num_points).copy()
        return array.view(np.float32).reshape(-1, 4)
    
    def encoding(self, array):
        self.pub_data.extend(array.ravel().T.tobytes())
    
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
    global prev_data_np

    pcd2mng = PointCloud2Manager(msg)
    pcd2mng.findField()

    rospy.loginfo(f"seqence {pcd2mng.header.seq}")
    start_time = rospy.Time.now()
    if pcd2mng.isOffset():
        sub_data_np = pcd2mng.decoding()

        sub_data_np = dynamic_detection(sub_data_np)

        pcd2mng.encoding(sub_data_np)

        pcd2mng.setPubPC2(height=len(sub_data_np) // 32, data=pcd2mng.pub_data)
        del pcd2mng
    end_time = rospy.Time.now()
    rospy.loginfo("Callback execution time: %f seconds" % (end_time - start_time).to_sec())


rospy.init_node('filter_node')

sub = rospy.Subscriber('rslidar_points', PointCloud2, callback)

pub = rospy.Publisher('filter_points', PointCloud2, queue_size=10)

rospy.spin()
