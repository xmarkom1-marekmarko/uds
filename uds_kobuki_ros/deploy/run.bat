@echo off

call c:\opt\ros\foxy\x64\setup.bat
call c:\opt\install\setup.bat
setx ROS_LOCALHOST_ONLY 1

ros2 run uds_kobuki_ros uds_kobuki_ros --ros-args -p ip_address:="127.0.0.1"
@REM  ros2 run uds_kobuki_ros uds_kobuki_ros --ros-args -p ip_address:="192.168.1.11"