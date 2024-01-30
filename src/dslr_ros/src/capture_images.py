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


def set_capture_target(camera, value):
    # get configuration tree
    config = gp.check_result(gp.gp_camera_get_config(camera))
    # find the capture target config item
    capture_target = gp.check_result(
        gp.gp_widget_get_child_by_name(config, "capturetarget")
    )
    # set value
    value = gp.check_result(gp.gp_widget_get_choice(capture_target, value))
    gp.check_result(gp.gp_widget_set_value(capture_target, value))
    # set config
    gp.check_result(gp.gp_camera_set_config(camera, config))
    return 0


def list_files(camera, path="/"):
    result = []
    # get files
    for name, value in camera.folder_list_files(path):
        result.append(os.path.join(path, name))
    # read folders
    folders = []
    for name, value in camera.folder_list_folders(path):
        folders.append(name)
    # recurse over subfolders
    for name in folders:
        result.extend(list_files(camera, os.path.join(path, name)))
    return result


def talker():
    rospy.init_node("capture_images", anonymous=True)

    pub = rospy.Publisher("/dslr/capture_msg", String, queue_size=10)
    session_name = rospy.get_param("session_name")

    final_path = f"/root/ros_ws/data/{session_name}/"
    if not os.path.exists(final_path):
        os.makedirs(final_path)

    camera = gp.Camera()
    camera.init()
    #    set_capture_target(camera, 1)
    event_texts = {
        gp.GP_EVENT_CAPTURE_COMPLETE: "Capture Complete",
        gp.GP_EVENT_FILE_ADDED: "File Added",
        gp.GP_EVENT_FOLDER_ADDED: "Folder Added",
        gp.GP_EVENT_TIMEOUT: "Timeout",
    }

    def event_text(event_type):
        if event_type in event_texts:
            return event_texts[event_type]
        return "Unknown Event"

    counter = 0
    while not rospy.is_shutdown():
        rospy.loginfo("Capturing image")
        file_path = camera.capture(
            gp.GP_CAPTURE_IMAGE,
        )

        print("Camera file path: {0}/{1}".format(file_path.folder, file_path.name))

        # save file
        save_file = camera.file_get(
            file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL
        )
        save_dest = os.path.join(final_path, file_path.name)
        print("Copying image to " + save_dest)
        save_file.save(save_dest)

        save_new_name = final_path + time.strftime("%Y%m%d%H%M%S.RAF", time.gmtime())
        os.rename(save_dest, save_new_name)

        # empty the event queue
        # typ, data = camera.wait_for_event(200)
        # while typ != gp.GP_EVENT_TIMEOUT:
        #     print("Event: %s, data: %s" % (event_text(typ), data))

        #     if typ == gp.GP_EVENT_FILE_ADDED:
        #         fn = os.path.join(data.folder, data.name)
        #         print("New file: %s" % fn)
        #         # self.download_file(fn)

        #     # try to grab another event
        #     typ, data = camera.wait_for_event(1)

        counter += 1

        if counter == 20:
            camera.folder_delete_all(file_path.folder)
            rospy.sleep(1)

    # pub.publish("Image Captured: " + save_dest)


def main():
    logging.basicConfig(
        format="%(levelname)s: %(name)s: %(message)s", level=logging.WARNING
    )
    callback_obj = gp.check_result(gp.use_python_logging())
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    sys.exit(main())
