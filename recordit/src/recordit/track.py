#!/usr/bin/env python
from tf import transformations as tfs


class Track(object):

    """
    A track object represents a movement of a frame over a series of keyframes.
    The movement can either be rotation(quaternion) or transformation(vector3).
    """

    def __init__(self, name, is_rot):
        self.name = name + ".quaternion" if is_rot else name + ".position"
        self.type = "quaternion" if is_rot else "vector3"
        self.keys, self.value = [], 0
        self.digits = 3  # digits to round all recorded values

    def r(self, _list):
        return [round(l, self.digits) for l in _list]

    def change_digits(self, digits):
        self.digits = digits

    def _update_value(self, v):
        self.value = self.r(v)

    def _add_kf(self, t):
        self.keys.append({"value": self.value, "time": round(t, self.digits)})

    def add(self, v, t, th):
        if self.keys and (self.keys[-1]["time"] < t - th):
            self._add_kf(t - th)
        self._update_value(v)
        self._add_kf(t)

    def export(self):
        return {p: getattr(self, p) for p in ("type", "name", "keys")}


class LinJTrack(Track):
    def __init__(self, name, j):
        Track.__init__(self, name, False)
        axis = j.axis if j.axis else [1, 0, 0]
        xyz = j.origin.xyz if j.origin else [0, 0, 0]
        self.vector = zip(xyz, axis)

    def _update_value(self, v):
        self.value = self.r([x + y * v for (x, y) in self.vector])


class RotJTrack(Track):
    def __init__(self, name, j):
        Track.__init__(self, name, True)
        ypr = j.origin.rpy[::-1] if j.origin else [0, 0, 0]
        self.axis = j.axis if j.axis else [1, 0, 0]
        self.q = tfs.quaternion_from_euler(*(ypr + ["rzyx"]))

    def _update_value(self, v):
        q_dyn = tfs.quaternion_about_axis(v, self.axis)
        self.value = self.r(tfs.quaternion_multiply(self.q, q_dyn).tolist())
