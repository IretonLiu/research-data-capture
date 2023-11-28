# dslr_ros
A ROS pkg for DSLR cameras using it as a webcam.

## Scripts
* ```sh 
  rosrun dslr_ros dslr2webcam.sh 
  ```
  Broadcasts dslr raw video on ```/dev/video0```. Then, the dslr camera can be used as a webcamera. Consequently, common ROS packages can be used for camera calibration, marker detection etc:
  1. [camera_calibration](http://wiki.ros.org/camera_calibration)
  2. [image_view](http://wiki.ros.org/image_view)
  3. [aruco_detect](http://wiki.ros.org/aruco_detect)

* Manual Calibration for full resolution DSLR image:
    1. ```sh
        rosrun dslr_ros capture_images.py 
        ``` 
        This script captures images every 4 seconds and saves them in ```/tmp/``` folder.
    

    2. ```sh
        rosrun dslr_ros calibration_module.py 
        ```
        This script gets the images and calculates the camera calibration matrix.
        ```
        optional arguments:
        -h, --help            show this help message and exit
        -cw CHESSBOARD_WIDTH, --chessboard_width CHESSBOARD_WIDTH
                        number of intersections in x axis
        -ch CHESSBOARD_HEIGHT, --chessboard_height CHESSBOARD_HEIGHT
                        number of intersections in y axis
        -sd SQUARE_DIMENSION, --square_dimension SQUARE_DIMENSION
                        square dimension in meters
        -p PATH, --path PATH  path to images folder
        -e FILE_EXTENSION, --file_extension FILE_EXTENSION
                        extension of images
        -a AUTO_MODE, --auto_mode AUTO_MODE
                        automatic mode uses all images inside images folder to
                        run calibration
        ```