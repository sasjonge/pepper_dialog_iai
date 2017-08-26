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
from dialog_system.msg import DialogResponses


class DialogCoreClientManager(object):
   def __init__(self):

        #constants to mark begin and end of discussions
        rospy.init_node('dialog_manager')
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting core server manager node...")
        #Publisher
        self.respub=rospy.Publisher('~dialog_responses',DialogResponses,queue_size=1000)
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
            resmsg = DialogResponses()
            msgcreated = False
            if (solution['Picture'] and solution['Picture'][0] != '_'):
                picmsg = DialogResponse()
                picmsg.type = DialogResponse.PICTURE
                picmsg.value = solution['Picture']
                msgcreated = True
                resmsg.responses.append(picmsg)
            if (solution['Answer'] and solution['Answer'][0] != '_'):
                ansmsg = DialogResponse()
                ansmsg.type = DialogResponse.ANSWER
                ansmsg.value = solution['Answer']
                msgcreated = True
                resmsg.responses.append(ansmsg)
            if (solution['Animation'] and solution['Animation'][0] != '_'):
                animsg = DialogResponse()
                animsg.type = DialogResponse.ANIMATION
                animsg.value = solution['Animation']
                resmsg.responses.append(animsg)
                msgcreated = True
            if msgcreated:
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
