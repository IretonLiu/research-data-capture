## ROS

Installing dependencies 

`rosdep install --from-paths src --ignore-src --rosdistro noetic -y`

Launching Velodyne

`roslaunch velodyne_pointcloud VLP16_points.launch`

Adding a transform from map to velodyne

`rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map velodyne 10`

Running rosbag

`rosbag record -a -o <session_name>`

Remember to index rosbag before playback

`rosbag reindex <bagfile>`
