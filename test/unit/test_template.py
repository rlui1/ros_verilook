import unittest
import concurrent.futures
import shutil
import tempfile
import json
from ros_verilook.identify import Template
from os.path import dirname, join, realpath
import os

class TestTemplate(unittest.TestCase):

    def setUp(self):
        dir_data = realpath(join(dirname(__file__), '..', 'data'))
        Template.DIR_TEMP = join(dir_data, 'templates/tmp')
        Template.DIR_SAVED = join(dir_data, 'templates/saved')

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

    def test_identify(self):
        matches = Template('523931f4b97bd8076c16dda77e7252bd').identify(self.executor)
        self.assertEquals([match.name for match in matches],
                          ['Gabrielius'])

    def tearDown(self):
        self.executor.shutdown()


class TestTemplateCopyData(unittest.TestCase):

    def setUp(self):
        dir_data = realpath(join(dirname(__file__), '..', 'data'))

        # Use a temporary location for memorized faces
        self.tempdir = tempfile.mkdtemp(prefix='ros_verilook_test_data_')
        os.rmdir(self.tempdir)
        shutil.copytree(dir_data, self.tempdir)
        Template.DIR_TEMP = join(self.tempdir, 'templates/tmp')
        Template.DIR_SAVED = join(self.tempdir, 'templates/saved')

        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

    def test_save(self):
        t = Template('523931f4b97bd8076c16dda77e7252bd')
        t.save('Test Name')

        # Check whether files were copied
        found_files = os.listdir(t.template_folder_path)
        expected_files = ['523931f4b97bd8076c16dda77e7252bd.template',
                          '523931f4b97bd8076c16dda77e7252bd.json',
                          '523931f4b97bd8076c16dda77e7252bd.jpg']
        self.assertEquals(set(found_files), set(expected_files))

        # Check whether json contains the given name
        with open(join(t.template_folder_path, '523931f4b97bd8076c16dda77e7252bd.json')) as f:
            self.assertEquals(json.load(f)['name'], 'Test Name')

    def tearDown(self):
        self.executor.shutdown()
        shutil.rmtree(self.tempdir)
