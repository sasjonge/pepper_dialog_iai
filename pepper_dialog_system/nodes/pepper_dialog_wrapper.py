#!/usr/bin/env python
import time
import rospy
import sys
from dialog_system.msg import DialogResponse
from dialog_system.msg import DialogResponses
from naoqi import (ALBroker, ALProxy, ALModule)
from std_msgs.msg import( String )
from naoqi_bridge_msgs.msg import(
    WordRecognized,
    )

#global variable
pd=None
image_instances=0

class Util:
    # Methods for name conversion
    @staticmethod
    def to_naoqi_name(name):
        return "ros{}_{}".format(
            name.replace("/", "_"),
            rospy.Time.now().to_sec() )



class Constants:
    EVENT = ["ALBasicAwareness/HumanTracked","ALBasicAwareness/HumanLost"]
    


class PepperDialogWrapper(ALModule):

    """ROS wrapper for Naoqi speech recognition"""
    def __init__(self):       
        #start the node
        rospy.init_node('pepper_dialog')

        #parameter reading
        self.PEPPERIP = rospy.get_param("PEPPERIP", "192.168.101.69")
        self.PEPPERPORT = int(rospy.get_param("PEPPERPORT", 9559))
        self.TABLETSLEEPTIME = int(rospy.get_param("TABLETSLEEPTIME", 10))
        #self.config=rospy.get_param("CONFIG", "")        
        # Get a (unique) name for naoqi module which is based on the node name
        # and is a valid Python identifier (will be useful later)
        self.naoqi_name = Util.to_naoqi_name( rospy.get_name() )

        #Start ALBroker (needed by ALModule)
        self.broker = ALBroker(self.naoqi_name + "_broker",
            "0.0.0.0",   # listen to anyone
             0,           # find a free port and use it
            self.PEPPERIP,          # parent broker IP
            self.PEPPERPORT )       # parent broker port
        
        #Init superclass ALModule
        ALModule.__init__( self, self.naoqi_name )
      
        # Start naoqi proxies
        self.memory = ALProxy("ALMemory",self.PEPPERIP,self.PEPPERPORT)
        #self.mic_spk = ALProxy("ALAudioDevice",self.PEPPERIP,self.PEPPERPORT)
        #self.proxy = ALProxy("ALSpeechRecognition",self.PEPPERIP,self.PEPPERPORT)
        self.al =ALProxy("ALAutonomousLife",self.PEPPERIP,self.PEPPERPORT)
        self.al.switchFocus('emptybehavior/behavior_1')
        self.basic_awareness = ALProxy("ALBasicAwareness", self.PEPPERIP,self.PEPPERPORT)
        self.motion = ALProxy("ALMotion", self.PEPPERIP,self.PEPPERPORT)
        
        #lock speech recognizer
        self.is_speech_reco_started = False
        self.is_busy = False
        #self.mic_spk.closeAudioInputs()
        #on stop for publisher
        rospy.on_shutdown(self.cleanup)

        #Install global variables needed by Naoqi
        self.install_naoqi_globals()

        #kill running modules  
        for i in range(len(Constants.EVENT)):
		#Check no one else is subscribed to this event
		subscribers = self.memory.getSubscribers(Constants.EVENT[i])
		if subscribers:
		    rospy.logwarn(Constants.EVENT[i]+" already in use by another node")
		    for module in subscribers:
		        self.stop(module,Constants.EVENT[i])
        #subscribe to different events
        self.memory.subscribeToEvent(
            Constants.EVENT[0],
            self.naoqi_name,
            self.on_human_detected.func_name)

        self.memory.subscribeToEvent(
             Constants.EVENT[1],
             self.naoqi_name,
             self.on_human_lost.func_name)

        self.tts = ALProxy("ALTextToSpeech", self.PEPPERIP, int(self.PEPPERPORT))
        self.tablet = ALProxy("ALTabletService", self.PEPPERIP, int(self.PEPPERPORT))
        self.animation = ALProxy("ALAnimationPlayer", self.PEPPERIP, int(self.PEPPERPORT))
         
        # Subscribe to the message topics
        rospy.Subscriber('/dialogManager/dialog_responses', DialogResponse, self.control)


    #speech controllers   
    def startSpeechRecognition(self):
        """ activate the speech recognition when people disappear"""
        if (not self.is_speech_reco_started) and (not self.is_busy):
            self.is_speech_reco_started = True
            rospy.set_param('ORDER',1)

        

    def stopSpeechRecognition(self):
        """ stop speech recognition if human speaker disapper """
        if self.is_speech_reco_started:
            self.is_speech_reco_started = False
            rospy.set_param('ORDER',0)
    

    #Event handlers   
    def on_human_lost(self, key, value, subscriber_id ):
        """ raised when people disappear"""
        self.stopSpeechRecognition()
        rospy.loginfo('Human likely lost')


    def on_human_detected(self, key, value, subscriber_id ):
        """ raised when people appear"""
        rospy.loginfo('Human likely detected: '+str(value))
        if value >= 0:  # found a new person
           #alert the dialog Manager to synchronize for communication
           self.startSpeechRecognition()
           rospy.loginfo('new Human likely detected')
        else:
           rospy.loginfo('Human likely lost')
           self.stopSpeechRecognition()


    # Install global variables needed for Naoqi callbacks to work
    def install_naoqi_globals(self):
        globals()[self.naoqi_name] = self
        globals()["memory"] = self.memory


    #stop subscribers to a particular events
    def stop(self,module,events):
        rospy.loginfo("Unsubscribing '{}' from NAO services".format(
            module))
        try:
            self.memory.unsubscribeToEvent( events, module )
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO services")
        rospy.loginfo("Shutting down running speech recognizer node...")


    def cleanup(self,module=None):
        if(module==None):
          module=self.naoqi_name
        rospy.loginfo("Unsubscribing '{}' from NAO services".format(
            module))
        try:
            self.memory.unsubscribeToEvent( Constants.EVENT[0], module )
            self.memory.unsubscribeToEvent( Constants.EVENT[1], module )
            #self.memory.unsubscribeToEvent( Constants.EVENT[2], module )
            #self.proxy.unsubscribe(module)
            self.basic_awareness.stopAwareness()
            #self.motion.rest()
            self.al.stopFocus('emptybehavior/behavior_1')
            self.broker.shutdown()
        except RuntimeError:
            rospy.logwarn("Could not unsubscribe from NAO services")
        rospy.loginfo("Shutting down speech recognizer node...")


    def show_image(self,image):
        global image_instances
        image_instances += 1
        self.tablet.showImage(image)
        rospy.sleep(TABLETSLEEPTIME)
        image_instances -= 1
        self.tablet.hideImage()


    #synthesize msg to action   
    def control(self, msg):
        rospy.loginfo('Controller process on response')
        self.stopSpeechRecognition()
        self.is_busy = True
        rospy.sleep(0.5)
        #speak
        for response in msg.responses:
            if (response.type == DialogResponse.PICTURE):
                thread = threading.Thread(target=self.show_image, args=(response.value))
                thread.start()
            if (response.type == DialogResponse.ANSWER):
                self.tts.say(response.value)
            if (response.type == DialogResponse.ANIMATION):
                self.animation.run(response.value)

        rospy.loginfo('Controller finished')
        rospy.sleep(0.5)
        self.is_busy = False
        self.startSpeechRecognition()


if __name__=="__main__":
      global pd  
      pd=PepperDialogWrapper()
      #start
      #sr.motion.wakeUp()
      pd.basic_awareness.setEngagementMode("FullyEngaged")
      pd.basic_awareness.setTrackingMode("MoveContextually")
      pd.basic_awareness.startAwareness()
      #loop
      rospy.spin()
      sys.exit(0)




