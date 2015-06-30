#!/usr/bin/python3
import sys, os, shutil
from os.path import join, splitext
import pexpect
import random
import concurrent.futures
import json

import rospy, rospkg
from ros_verilook.srv import CreateTemplate, Identify, IdentifyResponse, Save, SaveResponse
from ros_verilook.msg import Match

DIR_PKG = rospkg.RosPack().get_path('ros_verilook')

class Template:

    DIR_SAVED = join(DIR_PKG, 'data/templates/saved')
    DIR_TEMP = join(DIR_PKG, 'data/templates/tmp')

    BINARY_IDENTIFY = join(DIR_PKG, 'sdk/Tutorials/Biometrics/C/Identify/Identify')

    create_template_service = rospy.ServiceProxy('create_face_template',
        CreateTemplate)


    def __init__(self, handle, is_saved=False):
        """ Create an object reference to existing template files. """
        self.handle = handle
        self.is_saved = is_saved


    @classmethod
    def from_camera(cls):
        """ Create new template files from the camera feed.
        Return an object reference to those files and some additional detected features
        """
        # Create reference
        handle = '{:032x}'.format(random.getrandbits(128))
        new_template = cls(handle)

        # Create files
        os.makedirs(new_template.template_folder_path, exist_ok=True)
        try:
            response = cls.create_template_service(
                join(new_template.template_folder_path, handle)
            )
        except rospy.ServiceException:
            os.removedirs(new_template.template_folder_path)
            raise

        return (cls(handle), response.face_position)


    @property
    def template_folder_path(self):
        return join((self.DIR_SAVED if self.is_saved else self.DIR_TEMP), self.handle)

    @property
    def template_file_path(self):
        return join(self.template_folder_path, self.handle + '.template')

    @property
    def json_file_path(self):
        return join(self.template_folder_path, self.handle + '.json')


    def identify(self):
        """ Find and return saved templates that match this one. """

        # Create template object references to valid saved template files.
        saved = self._get_saved()
        if len(saved) == 0:
            # No saved templates means no matches
            return []
        paths = [template.template_file_path for template in saved]

        # Helper function for the concurrent execution
        def load_all_json_data(templates):
            for t in templates:
                t._load_json_data()

        # Run the 'identify' binary and load json data concurrently,
        # because both take some time to complete.
        futures = [executor.submit(self._run_identify_binary, paths),
                   executor.submit(load_all_json_data)]
        concurrent.futures.wait(futures, timeout=10)
        idx_scores = futures[0].result()

        # Return a list of Match objects
        matches = []
        for i, score in idx_scores:
            match = Match(handle=saved[i].handle,
                          name=saved[i].json_data['name'],
                          score=score)
            matches.append(match)
        return matches


    def save(self, name):
        """ Save this template and give it a name. """

        # Beware that self.is_saved influences self.template_folder_path.
        # Don't change this order of statements.
        old_folder_path = self.template_folder_path
        self.is_saved = True
        new_folder_path = self.template_folder_path

        # Move template files
        os.makedirs(self.DIR_SAVED, exist_ok=True)
        shutil.move(old_folder_path, new_folder_path)

        # Save the new name
        self.json_data = {'name': name}
        with open(self.json_file_path, 'w') as file:
            json.dump(self.json_data, file)


    def _load_json_data(self):
        with open(self.json.file_path, 'r') as file:
            self.json_data = json.load(file)


    @classmethod
    def _get_saved(cls):
        """ Create template object references to valid saved template files. """
        templates = []
        for dirpath, dirnames, filenames in os.walk(cls.DIR_SAVED):
            valid_handles = [splitext(name)[0] for name in filenames
                             if splitext(name)[1] == '.template'
                             and splitext(name)[0] + '.json' in filenames]

            templates.extend(Template(h, is_saved=True) for h in valid_handles)
        return templates


    def _run_identify_binary(self, paths):
        # Spawn VeriLook's 'Identify'.
        child = pexpect.spawnu('{0} {1}.template {2}'.format(
            self.BINARY_IDENTIFY,
            self.handle,
            ' '.join(paths)
        ))
        child.logfile = sys.stderr

        # Parse output
        results = []
        while True:
            code = child.expect(["matched with ID 'GallerySubject_([0-9]+)' with score '([0-9]+)'", pexpect.EOF])
            if code == 0:
                i, score = child.match.groups()
                i = int(i) - 1 # We want indices to start from 0
                score = int(score)
                results.append((i, score))
            else:
                break
        return results


def identify_service(req):
    template, face_position = Template.from_camera()
    matches = template.identify()
    response = IdentifyResponse(handle=template.handle,
                                face_position=face_position,
                                success=len(matches) > 0,
                                matches=matches)
    return response


def save_service(req):
    Template(req.handle, is_saved=False).save(req.name)
    return SaveResponse()


if __name__ == '__main__':
    # Used to call several blocking tasks in parallel to save time.
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)

    rospy.init_node('identify_face_node')
    rospy.Service('identify_face', Identify, identify_service)
    rospy.Service('save_face', Save, save_service)
    rospy.spin()
