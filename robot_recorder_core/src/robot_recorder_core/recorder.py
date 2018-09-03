#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
from urdf_parser_py.urdf import URDF
from tf import transformations as tfs
from std_srvs.srv import Trigger, TriggerResponse
import json


def round_list(input_list, ndigits = 4):
    return [ round(i, ndigits) for i in input_list ]

def _tf_2_list(tf, j_type):
    props = ('x', 'y', 'z')
    if j_type == "prismatic":
        tr = tf.transform.translation
    elif j_type == "revolute":
        tr = tf.transform.rotation
        props += ('w',)
    return [round(getattr(tr, k), 2) for k in props]


class Track(object):

    """
    A track object represents a relative movement between two frames.
    The movement can either be rotation(quaternion) or transformation(vector3).
    """

    def _init_(self, name, j_type, joint = None):
        if j_type == "revolute" or j_type == "continuous":
            self.name = name + ".quaternion"
            self.type = "quaternion"
            if joint:
                # quaternion_from_euler(yaw, pitch, roll, 'rzyx')
                q = tfs.quaternion_from_euler(*(joint.origin.rpy[::-1] + ['rzyx']))
                def cb(v):
                    q_dyn = tfs.quaternion_about_axis(v, joint.axis)
                    self.value = round_list( tfs.quaternion_multiply(q, q_dyn).tolist() )
        elif j_type == "prismatic":
            self.name = name + ".position"
            self.type = "vector3"
            if joint:
                orig = joint.origin.xyz
                def cb(v):
                    self.value = round_list( [(x + y*v) for x, y in zip(orig, joint.axis)] )
        else:
            rospy.loginfo("Joint of type %s not supported!", j_type)

        def pass_value(v): self.value = v
        self._update = cb if joint else pass_value
        self.keys = []

    def add_value_kf(self, time, value):
        self._update(value)
        self.add_kf(time)

    def add_kf(self, time):
        self.keys.append({"value": self.value, "time": time})

    def export(self):
        return {k: getattr(self,k) for k in ('type', 'name', 'keys')}


class Recorder(object):

    """
    A library to record a robot's movement and save it for web-visualization.
    It can process joint states and transformations(intended for mobile robots).
    The behaviour is exported as .json file to use for the 3D-library three.js.
    Manuel mode requires the start and stop of the recorder via ROS services.
    """
    j_types = ["prismatic", "revolute"]

    def _init_(self, freq, output_name, manual = False):
        self._cleanup()
        self.dt = 1/float(freq)
        self.output_name = output_name
        self.node = "Recorder"

        if not manual:
            rospy.on_shutdown(self.export_to_file)
            self._start()
        else:
            rospy.Service('~preconfigure', Trigger, self.preconfigure_cb)
            rospy.Service('~start', Trigger, self.start_cb)
            rospy.Service('~discard', Trigger, self.stop_cb)
            rospy.Service('~save', Trigger, lambda _: self.stop_cb(_, save=True))
        
        rospy.Service('~pause', Trigger, self.pause_cb)

    def preconfigure_cb(self, _):
        self._preconfigure()
        return TriggerResponse (True, self.node + " preconfigured!")

    def start_cb(self, _):
        if not self.active:
            self._start()
            return TriggerResponse (True, self.node + " started!")
        else:
            return TriggerResponse (False, self.node + " is already started!")

    def pause_cb(self, _):
        if self.active:
            # Pause the time
            if not self.paused:
                self.pause_t = rospy.get_rostime()
                response_msg = self.node + " paused!"
            else: 
                if self.start_t != None:
                    _paused_t = rospy.get_rostime() - self.pause_t
                    self.start_t += _paused_t.to_sec()
                response_msg = self.node + " unpaused!"
            self.paused = not self.paused
            rospy.loginfo(response_msg)
            return TriggerResponse (True, response_msg)
        else:
            return TriggerResponse (False, self.node + " is not started!")

    def stop_cb(self, _, save=False):
        print save
        if self.active:
            rospy.loginfo(self.node + " stopped!")
            if save:
                self.export_to_file()
            self._cleanup(reset_preconfig=False)
            return TriggerResponse (True, self.node + " stopped!")
        else:
            return TriggerResponse (False, self.node + " is not started!")


    def _start(self):
        if not self.preconfigured:
            self._preconfigure()
        self.preconfigured = True
        self.active = True
        rospy.loginfo(self.node + " started! Waiting for robot to move ...")

    def _cleanup(self, reset_preconfig=True):
        self.active = self.paused = False
        if reset_preconfig:
            self.preconfigured = False
        self.js_tracks = self.tf_tracks = False
        self.start_t = self.last_js_t = self.last_tf_t = None
        self.tracks = {}

    def export_to_file(self):
        self.active = False
        rospy.loginfo("Recording stopped.")
        duration = max(self.last_tf_t, self.last_js_t)
        if self.tracks:
            _tracks = [t.export() for t in self.tracks.itervalues()]
            _animation_name = self.output_name
            if "/" in _animation_name: # If absolute path only use filename
                _animation_name = _animation_name.rsplit('/',1)[1]
            if "." in _animation_name: # Remove the extension
                _animation_name = _animation_name.rsplit('.',1)[0]
            animation = { "duration": duration, "name": _animation_name, "tracks": _tracks}
            rospy.loginfo("\n %s", json.dumps(animation, indent=4, sort_keys=False) )
            with open(self.output_name, "w") as file:
                file.write( json.dumps(animation) )
            rospy.loginfo("Saved record to file %s!", self.output_name)
        else:
            rospy.loginfo("Nothing to record!")

    def _add_kf_if_pause(self, names, time, last_t):
        if (time - last_t > 3*self.dt):
            for n in names:
                self.tracks[n].add_kf(time - self.dt)

    def _preconfigure(self, key='robot_description'):
        rospy.Subscriber("tf_changes", tfMessage, self._tf_cb)
        if rospy.has_param(key):
            rospy.Subscriber("joint_states", JointState, self._joint_states_cb)
            self.j_map = URDF.from_parameter_server(key).joint_map    
            rospy.loginfo("Loading robot description.")     
            rospy.sleep(2.)
        else:
            rospy.logwarn("No robot description found. Joint states will not be recorded.")
        self.preconfigured = True

    def _joint_states_cb(self, data):
        # Joint states should yield the robot's joint movements
        if self.active and not self.paused:
            now = data.header.stamp.to_sec()

            #INIT
            if not self.js_tracks:
                rospy.loginfo("Robot moved: Recording joint states from %s", data.name)
                self.start_t = now
                for name in data.name:
                    joint = self.j_map[name]
                    self.tracks[name] = Track(name, joint.type, joint)
                self.js_tracks = True

            #UPDATE
            time = round(now - self.start_t, 2)
            if self.last_js_t:
                self._add_kf_if_pause(data.name, time, self.last_js_t)
            for i, name in enumerate(data.name):
                self.tracks[name].add_value_kf(time, data.position[i])

            self.last_js_t = time


    def _tf_cb(self, data):
        # TF should track the (mobile) robot in relation to the world
        if self.active and not self.paused:
            # By default one tf: map <--> base_link
            for tf in data.transforms:
                now = tf.header.stamp.to_sec()
                root = tf.child_frame_id[1:] # default: base_link

                #INIT
                if not self.tf_tracks:
                    rospy.loginfo("Robot moved: Recording tf from %s to %s", tf.header.frame_id, root)
                    self.start_t = now
                    for ty in self.j_types:
                        self.tracks[root+ty] = Track(root, ty)
                    self.tf_tracks = True

                #UPDATE
                time = round(now - self.start_t, 2)
                if self.last_tf_t:
                    self._add_kf_if_pause([root+ty for ty in self.j_types], time, self.last_tf_t)
                for ty in self.j_types:
                    self.tracks[root+ty].add_value_kf(time, _tf_2_list(tf, ty))

            self.last_tf_t = time
