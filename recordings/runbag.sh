rosbag play --clock -l $1.bag &
rosrun image_transport republish in:=/camera/image_raw compressed out:=/camera/image_raw raw
