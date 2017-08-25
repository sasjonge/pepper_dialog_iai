#!/usr/bin/env python

import socket
import sys
import rospy
import roslib
import threading
from std_msgs.msg import String
import subprocess
from json_prolog import json_prolog
from dialog_system.msg import DialogResponse

#Here is a client for core management of dialogue


SELF_CLIENT_SOCKET=''

class DialogCoreClientManager(object):
   def __init__(self):

        #constants to mark begin and end of discussions
        rospy.init_node('dialogManager')
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting core server manager node...")
        #Publisher
        self.respub=rospy.Publisher('~dialogResponse',DialogResponse,queue_size=1000)
        # Subscriber
        rospy.Subscriber('/rpc_server/recognition', String, self.process)
        #chartscript server
        rospy.loginfo("starting core server Manager ...")
        self.prolog = json_prolog.Prolog()


   def cleanup(self):
        rospy.loginfo("Shutting down core server manager node...")

   def process(self,msg):
        #system's reaction
        rospy.loginfo('before*********************')
        rospy.loginfo(msg.data)
        query = self.prolog.query('pepper_answer(\''+msg.data+'\',Answer,Animation,Picture)')

        for solution in query.solutions():
            if (solution['Picture'] and solution['Picture'][0] != '_'):
                resmsg = DialogResponse()
                resmsg.type = DialogResponse.PICTURE
                resmsg.value = solution['Picture']
                self.respub.publish(resmsg)
            if (solution['Answer'] and solution['Answer'][0] != '_'):
                resmsg = DialogResponse()
                resmsg.type = DialogResponse.ANSWER
                resmsg.value = solution['Answer']
                self.respub.publish(resmsg)
            if (solution['Animation'] and solution['Animation'][0] != '_'):
                resmsg = DialogResponse()
                resmsg.type = DialogResponse.ANIMATION
                resmsg.value = solution['Animation']
                self.respub.publish(resmsg)
            rospy.loginfo('Found prolog solution')

        #print Inputs and Outputs of core system manager

        rospy.loginfo('CORE SERVER INPUT: '+msg.data) 
		  

if __name__=="__main__":
    try:
        DialogCoreClientManager()
        rospy.loginfo('We came here')
        rospy.spin()
    except:
        e = sys.exc_info()[0]
        rospy.loginfo("Danger: An ERROR ocurred: %s" % e)
