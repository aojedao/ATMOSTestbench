current setups:

all in one: run_aruco_stack_tmux.sh
to kill: tmux kill-session -t aruco_stack


one by one:

python3 src/ros_package/src/cam_decoder/src/camera_direct.py

ros2 run aruco_pose_estimation aruco_node.py --ros-args   -p marker_size:=0.075   -p aruco_dictionary_id:="DICT_4X4_50"   -p image_topic:="/image_raw"   -p camera_info_topic:="/camera_info" -p pose_filter_alpha:=0.18 -p marker_hold_time_sec:=1.0

ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0


if wanna replay :  ros2 bag play rosbag2_2026_04_23-19_14_00/ --topic /camera_info /image_raw
