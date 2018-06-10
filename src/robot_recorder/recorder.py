#!/usr/bin/env python
from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState
from tf.msg import tfMessage

import json
from urdf_parser_py.urdf import URDF
from tf import transformations as tfs
from std_srvs.srv import Empty


class Recorder(object):

    """
    A library to record a robot's movement and save it for web-visualization.
    It can process joint states and transformations(intended for mobile robots).
    The behaviour is exported as .json file to use for the 3D-library three.js.
    Manuel mode requires the start and stop of the recorder via ROS services.
    """
    j_types = ["prismatic", "revolute"]

    def __init__(self, freq, output_name, manual = False):
        rospy.Subscriber("joint_states", JointState, self.__joint_states_cb)
        rospy.Subscriber("tf_changes", tfMessage, self.__tf_cb)

        if not manual:
            self.active = True
            rospy.on_shutdown(self.export_to_file)
        else:
            self.active = False
            rospy.Service('~start', Empty, self.start)
            rospy.Service('~stop', Empty, self.export_to_file)

        self.start_t = self.last_js_t = self.last_tf_t = None
        self.tracks = {}
        self.dt = 1/float(freq)
        self.out_name = output_name

    def start(self):
        self.active = True

    def export_to_file(self):
        self.active = False
        duration = max(self.last_tf_t, self.last_js_t)
        if self.tracks:
            _tracks = [t.export() for t in self.tracks.itervalues()]
            if "/" in self.out_name: # If absolute path only use filename
                animation_name = self.out_name.rsplit('/',1)[1]
            if "." in animation_name: # Remove the extension
                animation_name = animation_name.rsplit('.',1)[0]
            animation = { "duration": duration, "name": animation_name, "tracks": _tracks}
            print("\n %s", json.dumps(animation, indent=4, sort_keys=False) )
            with open(self.out_name, "w") as file:
                file.write( json.dumps(animation) )
            print("Saved record to file %s!", self.out_name)
        else: print("Nothing to record!")

    def __add_kf_if_pause(self, names, time, last_t):
        if last_t and (time - last_t > 3*self.dt):
            for n in names:
                self.tracks[n].add_kf(time - self.dt)

    def __load_joint_map(self):
        self.j_map = URDF.from_parameter_server().joint_map

    def __tf_2_list(self, tf, j_type):
        props = ('x', 'y', 'z')
        if j_type == "prismatic":
            tr = tf.transform.translation
        elif j_type == "revolute":
            tr = tf.transform.rotation
            props += ('w',)
        return [round(getattr(tr, k), 2) for k in props]


    def __joint_states_cb(self, data):
        # Joint states should yield the robot's joint movements
        if self.active:
            now = data.header.stamp.to_sec()

            #INIT
            if self.last_js_t is None:
                rospy.loginfo("Recording joint states from %s", data.name)
                if not self.start_t: self.start_t = now
                self.__load_joint_map()
                for name in data.name:
                    joint = self.j_map[name]
                    self.tracks[name] = Track(name, joint.type, joint)

            #UPDATE
            time = round(now - self.start_t, 2)
            self.__add_kf_if_pause(data.name, time, self.last_js_t)
            for i, name in enumerate(data.name):
                self.tracks[name].add_value_kf(time, data.position[i])

            self.last_js_t = time


    def __tf_cb(self, data):
        # TF should track the (mobile) robot in relation to the world
        if self.active:
            # By default one tf: map <--> base_link
            for tf in data.transforms:
                now = tf.header.stamp.to_sec()
                root = tf.child_frame_id[1:] # default: base_link

                #INIT
                if not self.last_tf_t:
                    rospy.INFO("Recording tf from %s to %s", tf.header.frame_id, root)
                    if not self.start_t: self.start_t = now
                    for ty in self.j_types:
                        self.tracks[root+ty] = Track(root, ty)

                #UPDATE
                time = round(now - self.start_t, 2)
                self.__add_kf_if_pause([root+ty for ty in self.j_types], time, self.last_tf_t)
                for ty in self.j_types:
                    self.tracks[root+ty].add_value_kf(time, self.__tf_2_list(tf, ty))

            self.last_tf_t = time


class Track(object):

    """
    A track object represents a relative movement between two frames.
    The movement can either be rotation(quaternion) or transformation(vector3).
    """

    def __init__(self, name, j_type, joint = None):
        if j_type == "revolute" or j_type == "continuous":
            self.name = name + ".quaternion"
            self.type = "quaternion"
            if joint:
                q = tfs.quaternion_from_euler(*(joint.origin.rpy + ['rxyz']))
                def cb(v):
                    q_dyn = tfs.quaternion_about_axis(v, joint.axis)
                    self.value = tfs.quaternion_multiply(q, q_dyn).tolist()
        elif j_type == "prismatic":
            self.name = name + ".position"
            self.type = "vector3"
            if joint:
                orig = joint.origin.xyz
                def cb(v):
                    self.value = [(x + y*v) for x, y in zip(orig, joint.axis)]
        else:
            print("Joint of type %s not supported!", j_type)
        def pass_value(v): self.value = v
        self.__update = cb if joint else pass_value
        self.keys = []

    def add_value_kf(self, time, value):
        self.__update(value)
        self.add_kf(time)

    def add_kf(self, time):
        self.keys.append({"value": self.value, "time": time})

    def export(self):
        return {k: getattr(self,k) for k in ('type', 'name', 'keys')}
