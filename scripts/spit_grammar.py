#!/usr/bin/python
import rospy
import os
import pipes
from actions import get_cmd
from parser import FnHintParser

if __name__ == "__main__":
    rospy.init_node('grammar_spit')
    cmd = FnHintParser()
    cmd.register_syntax(get_cmd()) 
    print cmd.export_jsgf()
    outfile = rospy.get_param("grammar_file", '~/grammar.fsg')
    os.system('echo ' + pipes.quote(cmd.export_jsgf()) + " | " +
              "sphinx_jsgf2fsg -jsgf /dev/stdin -fsg " + outfile + " 2> /dev/null")
