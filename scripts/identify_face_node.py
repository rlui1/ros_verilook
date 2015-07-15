#!/usr/bin/env python3
import rospy
import concurrent.futures
import threading
from copy import copy
import os

from ros_verilook.srv import Identify, IdentifyResponse, Save, SaveResponse
from ros_verilook.identify import Template
from std_srvs.srv import Empty, EmptyResponse


def identify_service(req):
    """ Try to identify the person in the image stream. Timeout can be given as
    an argument if there is a need to identify faces rapidly. """

    rospy.loginfo('Identify request')

    # Fill the 'response' with as much data as 'capture_and_identify' can
    # in the given timeout.
    response = IdentifyResponse()
    lock = threading.Lock()
    future = executor.submit(_capture_and_identify, response, lock)
    try:
        timeout = req.timeout if req.timeout > 0 else None
        future.result(timeout=timeout)
    except concurrent.futures._base.TimeoutError:
        pass

    # Use the response no matter if 'capture_and_identify' is finished or not.
    with lock:
        response = copy(response)

    rospy.loginfo('{} matches'.format(len(response.matches)))
    return response


def _capture_and_identify(response, lock):
    # Capture face from image stream
    template, face_position = Template.from_camera()

    if template == None:
        rospy.loginfo('Face capture error')
        return

    # Face captured, partially fill the response
    with lock:
        response.handle = template.handle
        response.face_position = face_position

    # Identify the captured face among the memorized ones
    matches = template.identify(executor)

    # Fill the response with 0 or more matches.
    with lock:
        response.matches = matches


def save_service(req):
    """ Memorize a captured face for future identification. """
    rospy.loginfo('Save request')
    try:
        Template(req.handle, is_saved=False).save(req.name)
    except FileNotFoundError:
        pass
    return SaveResponse()


def load_params():
    """ Configure the Template class from ROS parameters """
    # Save location for memorized templates
    data_dir = rospy.get_param("~data_dir", None)
    if data_dir:
        Template.DIR_TEMP = os.path.join(data_dir, 'tmp')
        Template.DIR_SAVED = os.path.join(data_dir, 'saved')

    # IP address of the machine, on which "run_pgd.sh start" was run
    Template.license_server_ip = rospy.get_param('vl_license_server', '127.0.0.1')
    return EmptyResponse()


if __name__ == '__main__':
    # Used to call several blocking tasks in parallel to save time.
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

    rospy.init_node('identify_face_node')
    load_params()
    rospy.Service('identify_face', Identify, identify_service)
    rospy.Service('save_face', Save, save_service)
    rospy.Service('~refresh_params', Empty, lambda req: load_params())
    rospy.spin()
