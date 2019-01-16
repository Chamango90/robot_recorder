#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
from std_srvs.srv import Trigger, TriggerResponse
from urdf_parser_py.urdf import URDF

from recordit.recorder import Recorder, sm
from recordit.track import Track, LinJTrack, RotJTrack


def resp(srv, *argv, **kwargs):
    def srv_wrapper(_):
        success, msg = srv(*argv, **kwargs)
        rospy.loginfo(msg)
        return TriggerResponse(success, msg)

    return srv_wrapper


class ROSRecorder(Recorder):

    """
    Uses the Record lib to record a robot's movement and maps it to ROS1.
    It can process joint states and transformations(intended for mobile robots).
    Recorder can be controlled via ROS services.
    """

    def __init__(self):
        manual = rospy.get_param("~manual", False)
        threshhold = rospy.get_param("~pause_threshold", 0.3)
        digits = rospy.get_param("~round_digits", 3)
        path = rospy.get_param("~path", None)
        Recorder.__init__(self, path, threshhold, digits, rospy.loginfo)
        self.j_map = None

        rospy.Service("~pause", Trigger, resp(self.pause, rospy.get_rostime))

        if manual:
            rospy.Service("~preconfigure", Trigger, resp(self.preconfigure))
            rospy.Service("~start", Trigger, resp(self.start))
            rospy.Service("~discard", Trigger, resp(self.stop))
            rospy.Service("~save", Trigger, resp(self.stop, save=True))
        else:
            self.auto_mode(on_shutdown=rospy.on_shutdown)

    @sm(requ=["UNCONF"], trans="CONF")
    def preconfigure(self, key="robot_description"):
        rospy.Subscriber("tf_changes", tfMessage, self.tf_callback)
        if rospy.has_param(key):
            rospy.Subscriber("joint_states", JointState, self.js_callback)
            self.j_map = URDF.from_parameter_server(key).joint_map
            rospy.loginfo("Loading robot description ...")
            rospy.sleep(2.0)
        else:
            rospy.logwarn(
                "No robot_description found. Joint_states will not be recorded."
            )

    @sm(requ=["RUNNING"])
    def js_callback(self, data):
        """
        Joint states yield the robot's joint movements
        """
        self.get_time(data)
        for i, key in enumerate(data.name):
            if not self.tracks.has_key(key):
                joint = self.j_map[key]
                if joint.type == "prismatic":
                    self.new_track(key, LinJTrack(key, joint))
                elif joint.type in ["revolute", "continuous"]:
                    self.new_track(key, RotJTrack(key, joint))
                else:
                    rospy.loginfo("Joint of type %s not supported!", joint.type)
                    continue
            self.append_to_track(key, data.position[i])

    @staticmethod
    def tf_items(tf):
        def obj_to_list(obj, props):
            return [getattr(obj, p) for p in props]

        yield obj_to_list(tf.translation, ("x", "y", "z"))
        yield obj_to_list(tf.rotation, ("x", "y", "z", "w"))

    @sm(requ=["RUNNING"])
    def tf_callback(self, data):
        """
        TF yields the movement of the (mobile) robot in relation to the world.
        Default is one tf: map <--> base_link
        """
        self.get_time(data.transforms[0])
        for tf in data.transforms:
            for item in self.tf_items(tf):
                name = tf.child_frame_id[1:]
                key = name + len(item)
                if not self.tracks.has_key(key):
                    self.new_track(key, Track(name, is_rot=len(item) == 4))
                    rospy.loginfo("Created track %s.", key)
                self.append_to_track(key, item)
