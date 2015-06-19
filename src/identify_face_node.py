#!/usr/bin/python2
import sys, os, shutil
import pexpect
import random
import rospy
import ros_verilook.srv

BIN_DIR = 'build/bin'
DATA_DIR = 'data'

class VisionException(Exception):
    pass

class Template:

    DIR_SAVED = "templates/saved"
    DIR_TEMP = "templates"

    BINARY_IDENTIFY = "bin/Identify"

    def __init__(self, handle):
        self.handle = handle

    @classmethod
    def from_camera(cls):
        create_template = rospy.ServiceProxy('create_template', ros_verilook.srv.CreateTemplate)
        handle = "{:032x}".format(random.getrandbits(128))
        create_template(os.path.join(cls.DIR_TEMP, handle))

        return cls(handle)

    def identify(self):
        try:
            gallery = [f for f in os.listdir(self.DIR_SAVED) if os.path.splitext(f)[1] == '.template']
            gallery_paths = [os.path.join(self.DIR_SAVED, file) for file in gallery]
            if len(gallery) == 0:
                raise OSError
        except OSError:
            # In case of an unexistant or empty template directory.
            return []

        # Spawn VeriLook's 'Identify'.
        child = pexpect.spawnu('{0} {1}.template {2}'.format(
            self.BINARY_IDENTIFY,
            self.handle,
            ' '.join(gallery_paths)
        ))
        child.logfile = sys.stderr

        # Parse output
        results = []
        while True:
            i = child.expect(["matched with ID 'GallerySubject_([0-9]+)' with score '([0-9]+)'", pexpect.EOF])
            if i == 0:
                results.append(child.match.groups())
            else:
                break
        results = [{'id': int(j), 'score': score, 'file': gallery[int(j)-1]} for j, score in results]
        return results

    def save(self, name):
        os.makedirs(self.DIR_SAVED, exist_ok=True)
        shutil.copyfile(
            os.path.join(self.DIR_TEMP, self.handle),
            os.path.join(self.DIR_SAVED, name)
        )


def identify_service(req):
    template = Template.from_camera()
    return ros_verilook.srv.IdentifyResponse(template.identify())

def save_service(req):
    Template(req.handle).save(req.name)


if __name__ == '__main__':
    rospy.init_node('identify_face_node')
    rospy.Service('identify_face', ros_verilook.srv.Identify, identify_service)
    rospy.Service('save_face', ros_verilook.srv.Save, save_service)
    rospy.spin()
