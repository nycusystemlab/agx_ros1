#!/usr/bin/env python3
import rospy
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def publish_dummy_cloud():
    rospy.init_node('dummy_lidar_publisher')
    pub = rospy.Publisher('/fake_lidar_points', PointCloud2, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    count = 0
    while not rospy.is_shutdown():
        points = []
        # 讓半徑隨時間變化 (呼吸效果)
        radius = 2.0 + math.sin(count * 0.1) * 1.0
        
        # 產生一個圓環狀的點雲
        num_points = 360
        for i in range(num_points):
            angle = math.radians(i * (360.0 / num_points))
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 0.0 # 都在同一平面
            
            # 將 XYZ 打包成二進位資料 (float32, float32, float32)
            points.append(struct.pack('fff', x, y, z))

        # 組合 Header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "dummy_lidar_frame"

        # 組合 PointCloud2 訊息
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = num_points
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12 # 4+4+4
        
        # [修正] 使用 len(points) 取得長度
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(points)

        pub.publish(cloud_msg)
        rospy.loginfo(f"Published fake lidar cloud with radius: {radius:.2f}")
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_dummy_cloud()
    except rospy.ROSInterruptException:
        pass