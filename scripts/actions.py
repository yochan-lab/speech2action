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

import rosplan_interface as planner
import state_machine.msg
from constructs import *
from parser import FnHintParser

name = "sprinkles"


tts = rospy.Publisher("tosay", std_msgs.msg.String)
loc = rospy.ServiceProxy('/get_location_list', GetLocationList)
locations = None
nicegeneric = S('please') | 'will you'
nicepre = nicegeneric | S('go ahead and')
nicepost = nicegeneric | (~S('right') + 'now')

master = FnHintParser()

face_last_seen = None
TIME_TOLERANCE = 5

def face_callback(*args, **kwargs):
   global face_last_seen
   face_last_seen = datetime.datetime.now() 

def face_active():
   if face_last_seen is None:
       return False
   return face_last_seen > datetime.datetime.now() - datetime.timedelta(seconds=TIME_TOLERANCE)

@master.register_fn(keywords='dir')
def go(place):
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
        self.goals = []
        self._feedback = state_machine.msg.interactFeedback()
        self._server = SimpleActionServer("interact",
                                          state_machine.msg.interactAction,
                                          execute_cb=self.execute_cb,
                                          auto_start = False)
        self._server.start()
	rospy.loginfo( "Interact Server started")

    def speech_callback(self, topic_data, parser):
        rospy.loginfo(topic_data)
        rospy.loginfo("============")
        rospy.loginfo(topic_data)
        if self.active:
            rospy.loginfo("Interpreting...")
            goal_s = parser.parse_and_run(topic_data.data)
            rospy.loginfo("Result: ", goal_s)
            if hasattr(goal_s, '__iter__'):
                self.goals.extend(goal_s)
            elif goal_s is not None:
                self.goals.append(goal_s)

    def execute_cb(self, goal):
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

        #self._server.publish_feedback(self._feedback)
        time.sleep(3)
        while face_active():
            rospy.loginfo(face_active())
            time.sleep(.5)
        if not self.active:
            # We're dead, don't send bad info
            return
        self.active = False

        rospy.loginfo("done interacting")
        res = state_machine.msg.interactResult()
        res.action = self.goals
        self.goals = []
        if res.action:
            self._server.set_succeeded(res)
        else:
            self._server.set_aborted()

def get_cmd():
    global locations
    rospy.wait_for_service('/get_location_list')
    locations = loc().output
    cmd = ~S('okay') + S(name) + ~nicepre + (
        ((S('move') | 'go' | 'drive') % 'go' +
         ((S('to') + ~S('the') +
          (reduce(lambda x, y: x | y, locations, S("starting_point")) % 'place')))) |
        (S('stop') | 'halt' | 'exit') % 'halt' |
        ((S('spin') % 'spin' | S('turn') % 'go') + (S('around') | 'left' | 'right') % 'direction') |
        ((S('say') | 'tell me' | 'speak' | 'what is' | 'what\'s') % 'say' + ~S('your') +
         (S('name') | 'identification' | 'id' | 'hello' | 'hi') % 'info') |
        (S("where are you going") % 'where')
    ) + (~nicepost)
    return cmd


if __name__ == '__main__':
    rospy.init_node('speech_interpreter')
    master.register_syntax(get_cmd())
    speech_topic = rospy.get_param('~speech_in', 'recognizer/speech')
    planner.init()
    srv = InteractServer(rospy.get_name())
    textin = rospy.Subscriber(speech_topic, std_msgs.msg.String, callback=srv.speech_callback, callback_args=master)
    rospy.spin() 

##
#  Old main code, supports exporting FSG syntax.
#
##

#if __name__ == '__main__':
#    argparser = argparse.ArgumentParser()
#    subparsers = argparser.add_subparsers(dest='cmd')
#
#    exportparser = subparsers.add_parser('export-grammar')
#    exportparser.add_argument('outfile', nargs='?', type=argparse.FileType('w'), default=sys.stdout)
#    exportparser.add_argument('--grammar-identifier', type=str, default='cmd')
#    exportparser.add_argument('--grammar-namespace', type=str, default='cmd')
#    exportparser.add_argument('--convert', nargs='?', type=str, const="/dev/stdout")
#
#    runparser = subparsers.add_parser('run')
#    runparser.add_argument('--speech-topic', nargs='?', type=str, default="recognizer/speech")
#
#    args = argparser.parse_args(filter(lambda arg: not arg.startswith('__'),
#                                       sys.argv[1:]))
#    if args.cmd == "export-grammar":
#        args.outfile.write(master.export_jsgf(identifier=args.grammar_identifier,
#                                              namespace=args.grammar_namespace))
#        args.outfile.close()
#
#        if args.convert:
#            os.system('echo ' + pipes.quote(master.export_jsgf(identifier=args.grammar_identifier,
#                                                               namespace=args.grammar_namespace)) + " | " +
#                      "sphinx_jsgf2fsg -jsgf /dev/stdin -fsg " + args.convert + " 2> /dev/null")
#
#    elif args.cmd == 'run':
#        rospy.init_node('speech_interpreter')
#        textin = rospy.Subscriber(args.speech_topic, std_msgs.msg.String, callback=speech_callback, callback_args=master)
#        rospy.spin()

