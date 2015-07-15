#!/usr/bin/env python3

import unittest
from threading import Event
from collections import OrderedDict
import itertools
import tempfile
import shutil
import functools
from os.path import dirname, join
import os
import subprocess
import signal

import rospy
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from ros_verilook.srv import Identify, Save

class TestSaveAndIdentify(unittest.TestCase):

    def setUp(self):
        # Services to test
        self.identify_service = rospy.ServiceProxy('/identify_face', Identify)
        self.save_service = rospy.ServiceProxy('/save_face', Save)

        # Will be used to synchronize identify service calls with timestamps
        # in the video stream.
        self.stamp_scanner = StampScanner(functools.partial(
            rospy.Subscriber, '/usb_cam/image_raw/compressed', CompressedImage))

        # Use a temporary location for memorized faces
        self.tempdir = tempfile.mkdtemp(prefix='ros_verilook_test_data_')
        rospy.set_param('/identify_face_node/data_dir', self.tempdir)
        refresh_params = rospy.ServiceProxy('/identify_face_node/refresh_params', Empty)
        refresh_params()

    def test_linas_gabrielius(self):
        # Video stream to play
        bagname = 'linas-gabrielius.bag'

        # Faces to expect
        self.stamp_scanner.set_stamps(
            [1436763906, 1436763911, 1436763919, 1436763923])
        expected_faces = ['Linas', 'Gabrielius', 'Linas', 'Gabrielius']

        # Launch rosbag
        cmd = ['rosbag', 'play', join(dirname(__file__), '..', 'data', bagname)]
        with PopenKillOnExit(cmd):
            # Identify faces 4 times. At specified timestamps.
            # We expect found_faces to become ['0','1','0','1'].
            found_faces = []
            counter = itertools.count()
            while len(found_faces) < len(expected_faces):
                # Wait until a timestamp marker is received
                self.stamp_scanner.wait(10)

                # Try to identify the face in current view
                found_faces.append(self._identify_or_save(next(counter)))

        # "value_insensitive" means that we don't care how the faces are named.
        # We care only that, in this case,
        # found_faces[0] == found_faces[2] and found_faces[1] == found_faces[3]
        self.assertEquals(value_insensitive(found_faces),
                          value_insensitive(expected_faces))

    def _identify_or_save(self, index):
        """ Try to identify the face in current view.
        Memorize the face if identification unsuccessful. """

        # Use timeout=3 seconds, because using our quick-paced bagfiles
        # we need to identify faces rapidly, i.e. if a match is not found,
        # decide so quicker.
        response = self.identify_service(timeout=3)

        if len(response.handle) == 0:
            # Face capture unsuccessful
            # Return a new generated name
            name = str(index)
        elif len(response.matches) == 0:
            # No match, memorize this face
            # Give it a new generated name
            name = str(index)
            self.save_service(handle=response.handle, name=name)
        else:
            # Identification successful
            # Retrieve the name
            name = response.matches[0].name

        return name

    def tearDown(self):
        shutil.rmtree(self.tempdir)
        self.stamp_scanner.shutdown()


class StampScanner:
    """
    StampScanner can be used to block execution until a specified
    timestamp is published on the given topic.
    """
    def __init__(self, partialsub):
        self.sub = partialsub(self._check_for_marks)
        self._event = Event()
        self._is_shutdown = False

    def set_stamps(self, stamps):
        """ Set a list of timestamps in seconds. """
        self.stamps = sorted(stamps, reverse=True)

    def wait(self, timeout=None):
        """ Block until one of the specified timestamps arrives. """
        if not self._event.wait(timeout=timeout):
            raise Exception('StampScanner timed out.')
        self._event.clear()

    def shutdown(self):
        """ Unregister the subscriber and make its handler ineffective. """
        if not self._is_shutdown:
            self.sub.unregister()
            self._is_shutdown = True

    def _check_for_marks(self, msg):
        """ ROS subscriber handler. """
        if self._is_shutdown:
            return
        if msg.header.stamp.secs < self.stamps[-1]:
            return

        # Notify the blocking thread.
        self.stamps.pop()
        self._event.set()
        if len(self.stamps) == 0:
            self.shutdown()


class PopenKillOnExit(subprocess.Popen):

    """ Popen with an extra feature. """

    def __init__(*args, **kwargs):
        # Set a session id to the process to be able to kill its spawned children.
        kwargs.update({'preexec_fn': os.setsid})
        subprocess.Popen.__init__(*args, **kwargs)

    def __exit__(self, type, value, tb):
        # Send SIGINT to all of the process group.
        os.killpg(self.pid, signal.SIGINT)
        subprocess.Popen.__exit__(self, type, value, tb)


def value_insensitive(lst):
    """ Utility function.
    E.g. value_insensitive(['x', 8, 'x', True]) == [0, 1, 0, 2]
    """
    ordered_unique = OrderedDict(zip(lst, itertools.repeat(True))).keys()
    unique_item_to_index = dict(zip(ordered_unique, itertools.count()))
    return [unique_item_to_index[item] for item in lst]


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_save_and_identify')
    rostest.rosrun('ros_verilook', 'test_save_and_identify', TestSaveAndIdentify)
