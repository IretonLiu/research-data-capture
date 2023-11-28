#!/usr/bin/env python

# python-gphoto2 - Python interface to libgphoto2
# http://github.com/jim-easterbrook/python-gphoto2
# Copyright (C) 2015-19  Jim Easterbrook  jim@jim-easterbrook.me.uk
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

import logging
import os
import sys
import gphoto2 as gp
import time

import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node("cpature_images", anonymous=True)

    pub = rospy.Publisher("/dslr/capture_msg", String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    session_name = rospy.get_param("session_name")

    save_to_folder = "/root/ros_ws/data"  # "/media/ireton/SR20
    final_path = f"/root/ros_ws/data/{session_name}/"
    if not os.path.exists(final_path):
        os.makedirs(final_path)

    while not rospy.is_shutdown():
        callback_obj = gp.check_result(gp.use_python_logging())
        camera = gp.Camera()
        camera.init()

        print("Capturing image")
        file_path = camera.capture(
            gp.GP_CAPTURE_IMAGE,
        )
        print(file_path.name)
        print("Camera file path: {0}/{1}".format(file_path.folder, file_path.name))

        save_dest = os.path.join("/root/ros_ws/data", file_path.name)
        print("Copying image to", save_dest)

        save_file = camera.file_get(
            file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL
        )
        save_file.save(save_dest)

        save_new_name = final_path + time.strftime("%Y%m%d%H%M%S.jpg", time.gmtime())
        os.rename(save_dest, save_new_name)

        pub.publish("Image Captured: " + save_new_name)
        rospy.sleep(5)


def main():
    logging.basicConfig(
        format="%(levelname)s: %(name)s: %(message)s", level=logging.WARNING
    )
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    sys.exit(main())
