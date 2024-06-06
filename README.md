## Commnunicating with the Husky directly
1. With the computer that you want to wirelessly connect to the husky, start a wifi hotspot with the ssid and password.

2. Connect to the husky using a wired connection and ssh into it.
Edit the `/etc/wpa_supplicant/wireless.conf` file to include the ssid and password of the wifi hotspot.

3. Restart the wpa_supplicant service:
```sh
ifconfig # to find the name of the wireless interface

sudo killall wpa_supplicant # kill the current wpa_supplicant process
sudo wpa_supplicant -B -i <wireless-interface> -c /etc/wpa_supplicant/wireless.conf # start a new wpa_supplicant process
```
4. Upon receiving a successful connection message, run the following command to get the ip address of the husky:
```sh
ifconfig
```
5. On the computer that you want to connect to the husky, ssh into the husky using the ip address obtained in the previous step:
```sh
ssh administrator@<ip-address> # password is clearpath
```

## Docker
Install docker and docker-compose.

### Build the docker image:
`docker-compose build`

### Start the docker container and an interactive shell:
`docker compose up -d && docker compose exec velodyne /bin/bash`


## ROS

### Installing dependencies 

First try:

`rosdep install --from-paths src --ignore-src --rosdistro noetic -y`

If that doesn't work, try:

`rosdep install --from-paths src/velodyne src/usb_cam src/velodyne_simulator --ignore-src --rosdistro noetic -y`

### Building the workspace

Make sure you are in the workspace directory, in this case `ros_ws` and run `catkin_make`

### Setup correct hostname

In `/etc/hosts` file, add the following line at the top:
`<ip-address> cpr-a200-0971`

The `ROS_MASTER_URI` and `ROS_IP` environment variables are set in the `docker-compose.yml` file
to allow the container to communicate with the husky. Verify that `ROS_MASTER_URI` is set to `http://cpr-a200-0971:11311`
and the `ROS_IP` is set to the container's ip address.

Inside the container, verify that communication with the husky is working by running:
`rostopic list`

Verify that the velodyne is publishing data by running:
`rostopic echo /velodyne_points`

### Adding a transform from map to velodyne
This ie required to visualize the point cloud in rviz. Run the following command to add a static transform from the map frame to the velodyne frame:

In the ssh terminal connected to the husky, run:

`rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map velodyne 10`

It's important to note that this must be running in the husky's terminal for the transform to be published.

### Visualizing the point cloud

Just run `rviz` in the container and add a point cloud display with the topic `/velodyne_points`

## IGNORE BELOW

### Launching Velodyne

`roslaunch velodyne_pointcloud VLP16_points.launch`

Running rosbag

`rosbag record -a -o <session_name>`

Remember to index rosbag before playback

`rosbag reindex <bagfile>`

