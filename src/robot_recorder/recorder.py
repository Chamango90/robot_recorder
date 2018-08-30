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

class Recorder(object):

    """
    A library to record a robot's movement and save it for web-visualization.
    It can process joint states and transformations(intended for mobile robots).
    The behaviour is exported as .json file to use for the 3D-library three.js.
    Manuel mode requires the start and stop of the recorder via ROS services.
    """
    j_types = ["prismatic", "revolute"]

    def __init__(self, freq, output_name, manual = False):
        self.active = self.preconfigured = self.paused = False
        self.start_t = self.last_js_t = self.last_tf_t = None
        self.tracks = {}
        self.dt = 1/float(freq)
        self.out_name = output_name

        if not manual:
            rospy.on_shutdown(self.export_to_file)
            self.start()
        else:
            rospy.Service('~preconfigure', Trigger, self.preconfigure_service)
            rospy.Service('~start', Trigger, self.start_service)
            rospy.Service('~stop', Trigger, self.stop_service)
        
        rospy.Service('~pause', Trigger, self.pause_service)

    def start(self):
        if not self.preconfigured: self.__start_subscribers()
        self.preconfigured = True
        self.active = True
        rospy.loginfo("Robot recorder started !")

    def preconfigure_service(self, request):
        self.__start_subscribers()
        self.preconfigured = True
        return TriggerResponse (True, "Robot recorder preconfigured !")

    def start_service(self, request):
        self.start()
        return TriggerResponse (True, "Robot recorder started !")

    def pause_service(self, request):
        if (self.start_t is not None): 
            # Pause the time
            if self.paused: 
                self.pause_t = rospy.get_rostime()
                response_msg = "Robot recorder paused !"
            else: 
                _paused_t = rospy.get_rostime() - self.pause_t
                self.start_t += _paused_t.to_sec()
                response_msg = "Robot recorder unpaused !"
        self.paused = not self.paused
        return TriggerResponse (True, response_msg)

    def stop_service(self, request):
        self.export_to_file()
        return TriggerResponse (True, "Robot recorder stopped !")

    def export_to_file(self):
        self.active = False
        rospy.loginfo("Recording stopped.")
        duration = max(self.last_tf_t, self.last_js_t)
        if self.tracks:
            _tracks = [t.export() for t in self.tracks.itervalues()]
            _animation_name = self.out_name
            if "/" in _animation_name: # If absolute path only use filename
                _animation_name = _animation_name.rsplit('/',1)[1]
            if "." in _animation_name: # Remove the extension
                _animation_name = _animation_name.rsplit('.',1)[0]
            animation = { "duration": duration, "name": _animation_name, "tracks": _tracks}
            rospy.loginfo("\n %s", json.dumps(animation, indent=4, sort_keys=False) )
            with open(self.out_name, "w") as file:
                file.write( json.dumps(animation) )
            rospy.loginfo("Saved record to file %s!", self.out_name)
        else: rospy.loginfo("Nothing to record!")

    def __add_kf_if_pause(self, names, time, last_t):
        if last_t and (time - last_t > 3*self.dt):
            for n in names:
                self.tracks[n].add_kf(time - self.dt)

    def __start_subscribers(self, key='robot_description'):
        rospy.Subscriber("tf_changes", tfMessage, self.__tf_cb)
        if rospy.has_param(key):
            rospy.Subscriber("joint_states", JointState, self.__joint_states_cb)
            self.j_map = URDF.from_parameter_server(key).joint_map    
            rospy.loginfo("Loading robot description.")     
            rospy.sleep(2.)
        else:
            rospy.logwarn("No robot description found. Joint states will not be recorded.")


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
        if self.active and not self.paused:
            now = data.header.stamp.to_sec()

            #INIT
            if self.last_js_t is None:
                rospy.loginfo("Recording joint states from %s", data.name)
                if not self.start_t: self.start_t = now
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
        if self.active and not self.paused:
            # By default one tf: map <--> base_link
            for tf in data.transforms:
                now = tf.header.stamp.to_sec()
                root = tf.child_frame_id[1:] # default: base_link

                #INIT
                if self.last_tf_t is None:
                    rospy.loginfo("Recording tf from %s to %s", tf.header.frame_id, root)
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
        self.__update = cb if joint else pass_value
        self.keys = []

    def add_value_kf(self, time, value):
        self.__update(value)
        self.add_kf(time)

    def add_kf(self, time):
        self.keys.append({"value": self.value, "time": time})

    def export(self):
        return {k: getattr(self,k) for k in ('type', 'name', 'keys')}
