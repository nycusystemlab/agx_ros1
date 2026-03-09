# 鍵盤控制
``` bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
rosrun six_wheels_teleop imu_teletop_0908
```
# lidar建圖
```bash
roslaunch velodyne_pointcloud VLP16_points.launch
```
# hdl定位
```bash
roslaunch hdl_localization hdl_localization.launch
```

# realsense啟動
``` bash
roslaunch realsense2_camera rs_camera.launch
# 濾波
rosrun imu_filter_madgwick imu_filter_node   _use_mag:=false   _remove_gravity_vector:=true   _output_rate:=100.0   /imu/data_raw:=/camera/imu
```