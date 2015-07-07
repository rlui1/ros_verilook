#!/usr/bin/python3
import sys, os, shutil
from os.path import join, splitext
import pexpect
import random
import concurrent.futures
import json

import rospy, rospkg
from ros_verilook.srv import CreateTemplate
from ros_verilook.msg import Match

DIR_PKG = rospkg.RosPack().get_path('ros_verilook')

class Template:

    DIR_SAVED = join(DIR_PKG, 'data/templates/saved')
    DIR_TEMP = join(DIR_PKG, 'data/templates/tmp')

    # Location of used Verilook binaries
    BINARY_IDENTIFY = join(DIR_PKG, 'sdk/Tutorials/Biometrics/C/Identify/Identify')
    LD_LIBRARY_PATH = join(DIR_PKG, 'sdk/Lib/Linux_x86_64')

    create_template_service = rospy.ServiceProxy('create_face_template', CreateTemplate)
    license_server_ip = rospy.get_param('vl_license_server', '127.0.0.1')


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


    def identify(self, executor):
        """ Find and return saved templates that match this one.

        The argument 'executor' should be a thread pool. It will be used to call
        several blocking tasks in parallel to save time. """

        # Create template object references to valid saved template files.
        saved = self._get_saved()
        if len(saved) == 0:
            # No saved templates means no matches
            return []
        paths = [template.template_file_path for template in saved]

        # Helper function for the concurrent execution
        def load_all_json_data():
            for t in saved:
                t._load_json_data()

        # Run the 'identify' binary and load json data concurrently,
        # because both take some time to complete.
        futures = [executor.submit(self._run_identify_binary, paths),
                   executor.submit(load_all_json_data)]
        concurrent.futures.wait(futures, timeout=10)

        # Forward any exceptions from child tasks
        for exception in [future.exception() for future in futures]:
            if exception:
                raise exception

        # Get the result of self._run_identify_binary
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
        with open(self.json_file_path, 'r') as file:
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
        child = pexpect.spawnu('{} {} {} {}'.format(
            self.BINARY_IDENTIFY,
            self.license_server_ip,
            self.template_file_path,
            ' '.join(paths)
        ), env = {'LD_LIBRARY_PATH': self.LD_LIBRARY_PATH})
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
