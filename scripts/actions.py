#!/usr/bin/python
import argparse
import os
import pipes
import sys

import rospy
import std_msgs.msg
import actionlib
import time

import datetime
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib import *
from location_provider.srv import GetLocationList
from name_provider.srv import GetRealName, CreateNewPerson
from std_srvs.srv import Trigger

try:
    import rosplan_interface as planner
    import state_machine.msg
except:
    pass
from constructs import *
from parser import FnHintParser

name = "sprinkles"


tts = rospy.Publisher("tosay", std_msgs.msg.String, queue_size=5)
loc = rospy.ServiceProxy('/get_location_list', GetLocationList)
locations = None
nicegeneric = S('please') | 'will you'
nicepre = nicegeneric | S('go ahead and')
nicepost = nicegeneric | (~S('right') + 'now')

master = FnHintParser()

face_last_seen = None
currently_speaking = False
TIME_TOLERANCE = 5

def voice_callback(msg):
    global currently_speaking 
    if not msg.data:
	time.sleep(2)
    currently_speaking = msg.data

def face_callback(_):
    global face_last_seen
    face_last_seen = datetime.datetime.now() 

def face_active():
    if face_last_seen is None:
        return False
    active = face_last_seen > datetime.datetime.now() - datetime.timedelta(seconds=TIME_TOLERANCE)
#    rospy.loginfo("Face detected? %s" % str(active))
    return active

@master.register_fn(keywords='dir')
def go(place):
    global currently_speaking
    currently_speaking = True
    _say("Okay. I will go to the " + str(place))
    return planner.gen_predicate('robotat', x=place)


@master.register_fn()
def say(info):
    msg = ""
    if info == "name":
        msg = "I am " + name
    elif info in ['identification', 'id']:
        msg = "I am a people bot"
    elif info in ['hello', 'hi']:
        msg = "Hello there."
    elif info  == "love":
        msg = "Not particularly"
    elif info == "friend":
        msg = "I am a people bot. I am a friend to everyone." 
    elif info == 'joke':
	msg = os.popen('fortune riddles | sed "s/Q://" | sed "s/A://" | tr "\n\t" " "').read()
	

    if msg:
        _say(msg)

def _say(msg):
    rospy.loginfo("Saying: " + msg)
    tts.publish(std_msgs.msg.String(msg))


@master.register_fn()
def halt():
    planner.cancel()
    _say("Stopping")


@master.register_fn()
def spin(direction='around'):
    _say('No')
    return


class InteractServer(object):
    def __init__(self, name):
        self.active = False
        self._action_name = name
	self.name = None
        self.goals = []
        self._feedback = state_machine.msg.interactFeedback()
        self._server = SimpleActionServer("interact",
                                          state_machine.msg.interactAction,
                                          execute_cb=self.execute_cb,
                                          auto_start = False)
        self._server.start()
	rospy.loginfo( "Interact Server started")

    def speech_callback(self, topic_data, parser):
        rospy.loginfo("============")
        rospy.loginfo('%s, speaking: %s' % (topic_data.data, str(currently_speaking)))
        if self.active and not currently_speaking:
            rospy.loginfo("Interpreting...")
            goal_s = parser.parse_and_run(topic_data.data)
            rospy.loginfo("Result: %s", str(goal_s))
            if hasattr(goal_s, '__iter__'):
                self.goals.extend(goal_s)
            elif goal_s is not None:
                self.goals.append(goal_s)
        rospy.loginfo("============")

    def check_name(self, id):
	self.name = None
	rospy.loginfo("Checking name... %d" % id)
	if not hasattr(self, '_name_service'):
	    self._name_service = rospy.ServiceProxy('/get_real_name', GetRealName)
	try:
	    rospy.wait_for_service('/get_real_name', 10)
	except:
	    rospy.logwarn("Timeout waiting for person db")
	    return
	res = self._name_service(id)   
	if res.found_name:
	    self.name = res.name
	    _say("Hello %s." % self.name)
	else:
	    _say("I do not recognize you.")
	    self.set_name(id)

    @master.register_fn()
    def set_name(self, id=None, *args, **kwargs):
	if id is None:
	    id = self.id
	if not hasattr(self, '_nl_service'): 
	    self._nl_service = rospy.ServiceProxy('/nl_recognizer/nl_listen', Trigger)
	if not hasattr(self, '_mkname_service'): 
	    self._mkname_service = rospy.ServiceProxy('/create_new_person', CreateNewPerson)
	try:
	    rospy.wait_for_service('/nl_recognizer/nl_listen', 10)
	except:
	    rospy.logwarn("Timeout waiting for listener")
	    return False
	try:
	    rospy.wait_for_service('/create_new_person', 10)
	except:
	    rospy.logwarn("Timeout waiting for person creation")
	    return False
	_say("Please state your name.")
	rospy.sleep(5)
	newname = self._nl_service()
	if newname.success:
	    self._mkname_service(newname.message, id)
	    self.name = newname.message
	    _say("Hello %s." % self.name)
	    return newname.message
	else:
	    _say("I didn't catch that. Continuing on.")
	    return False


    def execute_cb(self, goal):
	self.id = goal.personID
        #print goal.goal_id
        if self.active:
            if False and self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._server.set_preempted()
                self.active = False
            return
        rospy.loginfo("interacting")
        self.active = True
        #self._feedback.isInteracting = True
	# _say("Hello there.")
	self.check_name(goal.personID)
	_say("Please say a command")
        #self._server.publish_feedback(self._feedback)
        time.sleep(3)
        while face_active():
            time.sleep(.5)
        if not self.active:
            # We're dead, don't send bad info
            return
        self.active = False

        rospy.loginfo("done interacting")
        res = state_machine.msg.interactResult()
        res.action = self.goals
        self.goals = []
	_say('Goodbye')
        if res.action:
            self._server.set_succeeded(res)
        else:
            self._server.set_aborted()

def get_cmd():
    global locations
    rospy.wait_for_service('/get_location_list')
    locations = loc().output
    cmd = ~S('okay') + S(name) + ~nicepre + (
	(S('change my name') % 'set_name') |
        ((S('move') | 'go' | 'drive') % 'go' +
         ((S('to') + ~S('the') +
          (reduce(lambda x, y: x | y, locations, S(locations.pop())) % 'place')))) |
        (S('stop') | 'halt' | 'exit') % 'halt' |
        ((S('spin') % 'spin' | S('turn') % 'go') + (S('around') | 'left' | 'right') % 'direction') |
        ((S('say') | 'tell me' | 'speak' | 'what is' | 'what\'s') % 'say' + ~(S('your') | 'a') +
         (S('name') | 'identification' | 'id' | 'hello' | 'hi' | 'joke') % 'info') |
        (S("where are you going") % 'where')
    ) + (~nicepost)
    return cmd


if __name__ == '__main__':
    rospy.init_node('speech_interpreter')
    master.register_syntax(get_cmd())
    speech_topic = rospy.get_param('~speech_in', '/recognizer/output')
    active_topic = rospy.get_param('~active', '/face_finder/closest_face')
    planner.init()
    srv = InteractServer(rospy.get_name())
    textin = rospy.Subscriber(speech_topic, std_msgs.msg.String, callback=srv.speech_callback, callback_args=master)
    check_active = rospy.Subscriber(active_topic, rospy.msg.AnyMsg, callback=face_callback)
    check_voice = rospy.Subscriber('/is_speaking', std_msgs.msg.Bool, callback=voice_callback)
    rospy.spin() 

