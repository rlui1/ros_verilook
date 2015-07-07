#!/usr/bin/python3
import rospy
import concurrent.futures
from ros_verilook.srv import Identify, IdentifyResponse, Save, SaveResponse
from ros_verilook.identify import Template


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


if __name__ == '__main__':
    # Used to call several blocking tasks in parallel to save time.
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

    rospy.init_node('identify_face_node')
    rospy.Service('identify_face', Identify, identify_service)
    rospy.Service('save_face', Save, save_service)
    rospy.spin()
