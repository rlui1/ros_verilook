#!/usr/bin/python3
import rospy
import concurrent.futures
import os

from ros_verilook.srv import Identify, IdentifyResponse, Save, SaveResponse
from ros_verilook.identify import Template
from std_srvs.srv import Empty, EmptyResponse


def identify_service(req):
    """ Try to identify the person in the image stream. """
    template, face_position = Template.from_camera()
    matches = template.identify(executor)
    response = IdentifyResponse(handle=template.handle,
                                face_position=face_position,
                                matches=matches)
    return response


def save_service(req):
    """ Memorize a captured face for future identification. """
    Template(req.handle, is_saved=False).save(req.name)
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
