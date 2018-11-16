#!/usr/bin/env python
PKG = "recordit"
NAME = "test_recordit"

import rospy
import unittest, rostest
import json, os, sys
from math import pi
from sensor_msgs.msg import JointState
from recordit.client import Recorder

# General props for a .json animation
track_props = ("type", "keys", "name")
key_props = ("value", "time")
t_types = ("quaternion", "vector3")

# Specific to robot and trajectory of this test
j_names = ["lin_joint", "rot_joint"]
test_traj = [{"t": 0, "v": [0, 0]}, {"t": 0.5, "v": [1, pi]}]
results = [[[1, 1, 0], [1, 2, 0]], [[0, 0, 0, 1], [1, 0, 0, 0]]]


def err(msg):
    return {"msg": "ERROR: %s wrong!" % msg}


class CheckRecordedFile(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NAME)
        self.path = rospy.get_param("~path", "record.json")
        self.threshhold = rospy.get_param("~pause_threshold", 0.3)
        self.places = 2
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.js_msg = JointState(name=j_names)
        self._del_file()
        self._record_to_file()
        self._add_wait_kf()

    def _del_file(self):
        try:
            os.remove(self.path)
        except OSError:
            pass

    def _record_to_file(self):
        with Recorder("robot_recorder"):
            rospy.sleep(0.1)
            for p in test_traj:
                rospy.sleep(p["t"])
                self._publish_js(p["v"])

    def _add_wait_kf(self):
        global test_traj, results
        t = test_traj[-1]["t"] - self.threshhold
        test_traj = test_traj[:1] + [{"t": t}] + test_traj[1:]
        results = [r[:1] + r for r in results]

    def _publish_js(self, j_val):
        self.js_msg.header.stamp = rospy.Time.now()
        self.js_msg.position = j_val
        self.pub.publish(self.js_msg)

    def test_recorded_file(self):
        with open(self.path) as actual:
            animation = json.load(actual)
            tracks = animation["tracks"]
            self.assertEqual(len(tracks), len(j_names), **err("Track length"))
            for i, t in enumerate(tracks):
                self.check_track(i, t)

    def check_track(self, i, track):
        rospy.loginfo("Test track #%i: %s" % (i, track))
        self.assertTrue(
            all(p in track for p in track_props), **err("Track props (%i)" % i)
        )

        _type = track["type"]
        self.assertTrue(_type in t_types, **err("Track type (%i)" % i))

        _n = j_names[i]
        _n += ".quaternion" if _type == "quaternion" else ".position"
        self.assertEqual(track["name"], _n, **err("Track name (%s)" % _n))

        _keys = track["keys"]
        self.assertEqual(len(_keys), len(test_traj), **err("Key length (%s)" % _n))
        for j, key in enumerate(_keys):
            self.assertTrue(all(p in key for p in key_props), **err("Key props"))
            self.assertList(key["value"], results[i][j], self.places)
            self.assertAlmostEqual(
                key["time"],
                test_traj[j]["t"],
                self.places,
                **err("Time of key (%s, %i)" % (_n, j))
            )

    def assertList(self, list1, list2, places):
        for l1, l2 in zip(list1, list2):
            rospy.loginfo("Compare %i to %i" % (l1, l2))
            self.assertAlmostEqual(l1, l2, places, **err("Key value"))


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, CheckRecordedFile, sys.argv)
