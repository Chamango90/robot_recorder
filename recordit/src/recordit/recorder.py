#!/usr/bin/env python
import json
from time import strftime
import os
from functools import wraps
from enum import Enum


def statemachine_decorator(requ, trans=None):

    """
    Method decorator for a simple State Machine.
    Requires properties State and States(enum).
    """

    def check_for_state(method):
        @wraps(method)
        def method_wrapper(self, *argv, **kwargs):
            if any(self.states["UNCONF"] == self.states[r].value for r in requ):
                return (False, "State must be in %s!" % requ)
            method(self, *argv, **kwargs)
            if trans:
                self.state = self.states[trans]
                return (True, "State changed to %s." % trans)
            return (True, "")

        return method_wrapper

    return check_for_state


sm = statemachine_decorator


def parse_path(path):
    rel_p, f_name = os.path.split(path)
    name, f_ext = os.path.splitext(f_name)
    rel_p = rel_p or os.getcwd()
    name = name or strftime("recording-%Y-%m-%d-%H-%M-%S")
    f_ext = f_ext or ".json"
    return name, os.path.join(rel_p, name + f_ext)


class Recorder(object):

    """
    A library to record a robot's movement and save it for web-visualization.
    The behaviour is exported as .json file to use for the 3D-library three.js.
    Can either be run in auto or manuel mode.
    """

    def __init__(self, path, threshhold, digits, INFO):
        self.name, self.path = parse_path(path)
        self.threshhold, self.digits = threshhold, digits
        self.states = Enum("States", "UNCONF CONF RUNNING PAUSED")
        self.state = self.states.UNCONF
        self.start_t = self.now = self.pause_t = None
        self.tracks = {}
        self.INFO = INFO  # log function

    def auto_mode(self, on_shutdown):
        on_shutdown(self.export_to_file)
        self.start()

    @sm(requ=["UNCONF"], trans="CONF")
    def preconfigure(self):
        pass

    @sm(requ=["UNCONF", "CONF"], trans="RUNNING")
    def start(self):
        if self.state == self.states.UNCONF:
            self.preconfigure()
        self.INFO(" Waiting for robot to move ...")

    def pause(self, t):
        t = t().to_sec()
        if self.state == self.states.RUNNING:
            return self._pause(t)
        elif self.state == self.states.PAUSED:
            return self._unpause(t)
        else:
            return (False, "Recorder not started!")

    @sm(requ=["RUNNING"], trans="PAUSED")
    def _pause(self, t):
        self.pause_t = t

    @sm(requ=["PAUSED"], trans="RUNNING")
    def _unpause(self, t):
        if self.start_t != None:
            self.start_t += t - self.pause_t

    @sm(requ=["RUNNING", "PAUSED"], trans="CONF")
    def stop(self, save=False):
        if save:
            self.export_to_file()
        self.start_t = self.now = None
        self.tracks = {}

    def get_time(self, root):
        now = root.header.stamp.to_sec()
        if self.start_t is None:
            self.INFO("Robot moved: Start recording!")
            self.start_t = now
        self.now = now - self.start_t

    def append_to_track(self, key, v):
        self.tracks[key].add(v, self.now, self.threshhold)

    def new_track(self, key, track):
        self.tracks[key] = track
        self.tracks[key].change_digits(self.digits)
        self.INFO("Created track %s.", key)

    def get_animation(self):
        if not self.tracks:
            return None
        return {
            "duration": self.now,
            "name": self.name,
            "tracks": [t.export() for t in self.tracks.itervalues()],
        }

    @sm(requ=["RUNNING", "PAUSED"], trans="CONF")
    def export_to_file(self):
        anim = self.get_animation()
        if not anim:
            self.INFO("Nothing to record!")
        else:
            self.INFO("Recorded: \n %s" % json.dumps(anim, indent=4))
            with open(self.path, "w") as _file:
                _file.write(json.dumps(anim))
                self.INFO("Saved record to file %s!" % self.path)
