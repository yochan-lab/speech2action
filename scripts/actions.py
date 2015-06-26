#!/usr/bin/python
import argparse
import os
import pipes
import sys

import rospy
import std_msgs.msg
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from constructs import *
from parser import FnHintParser

name = "sprinkles"


move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
locations = {"tony's office": Pose(Point(2.495, 1.964, 0.000), Quaternion(0.000, 0.000, -0.044, 0.999)),
             "room five sixty": Pose(Point(2.581, 4.985, 0.000), Quaternion(0.000, 0.000, -0.028, 1.000)),
             "kitchen": Pose(Point(-8.588, 28.927, 0.000), Quaternion(0.000, 0.000, 1.000, -0.015)),
             "cubicle": Pose(Point(-0.717, 0.897, 0.000), Quaternion(0.000, 0.000, 0.734, 0.680)),
             "conference room": Pose(Point(1.056, 35.795, 0.000), Quaternion(0.000, 0.000, 0.048, 0.999)),
             "eye tea": Pose(Point(-15.275, 36.182, 0.000), Quaternion(0.000, 0.000, 0.741, 0.672)),
             "entrance": Pose(Point(-4.926, 5.199, 0.000), Quaternion(0.000, 0.000, 0.977, -0.211))}
tts = rospy.Publisher("tosay", std_msgs.msg.String)

nicegeneric = S('please') | 'will you'
nicepre = nicegeneric | S('go ahead and')
nicepost = nicegeneric | (~S('right') + 'now')
cmd = S('okay') + S(name) + ~nicepre + (
    ((S('move') | 'go' | 'drive') % 'go' +
     ((S('right') | 'left') % 'direction' | (S('to') + ~S('the') +
      (reduce(lambda x, y: x | y, locations.iterkeys(), S("nowhere")) % 'place')))) |
    (S('stop') | 'halt' | 'exit') % 'halt' |
    ((S('spin') % 'spin' | S('turn') % 'go') + (S('around') | 'left' | 'right') % 'direction') |
    ((S('say') | 'tell me' | 'speak' | 'what is' | 'what\'s') % 'say' + ~S('your') +
     (S('name') | 'identification' | 'id' | 'hello' | 'hi') % 'info') |
    (S("where are you going") % 'where') |
    (S('are you my') + (S("friend") % 'info')) |
    (S('do you') + (S("love") % 'info') + "me")
) + (~nicepost)

master = FnHintParser()


@master.register_fn(keywords='dir')
def go(place):
    move_base.cancel_all_goals()
    if place in locations:
        goal = MoveBaseGoal()
        goal.target_pose.pose = locations[place]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        move_base.send_goal(goal)
        _say("Now going to the " + str(place))
    elif place == "nowhere":
        _say("We're getting nowhere. Stopping.")
    else:
        _say("I don't know where " + location + " is.")


@master.register_fn()
def go_to(direction):
    _say("Direct control via voice is a really bad idea")


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
    move_base.cancel_all_goals()
    _say("Stopping")


@master.register_fn()
def spin(direction='around'):
    return




master.register_syntax(cmd)





def speech_callback(topic_data, parser):
    rospy.loginfo("============")
    rospy.loginfo(topic_data)
    parser.parse_and_run(topic_data.data)


if __name__ == '__main__':
    rospy.init_node('speech_interpreter')
    speech_topic = rospy.get_param('~speech_in', 'recognizer/speech')
    textin = rospy.Subscriber(speech_topic, std_msgs.msg.String, callback=speech_callback, callback_args=master)
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

