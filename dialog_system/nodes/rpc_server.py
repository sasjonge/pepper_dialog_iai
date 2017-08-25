#!/usr/bin/env python
from SimpleXMLRPCServer import  SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
import rospy
import roslib
from std_msgs.msg import String
import utility as u
# Create ros-server-node and rpc-server for Pepper 

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)
requestHandler=RequestHandler
class Server:
   def __init__(self):
        rospy.init_node('server')
        self.confirmation="0"
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Starting server node...")
        #read the parameter
        self.RPCSERVERIP = rospy.get_param("HOST", "192.168.178.38")
        self.RPCSERVERPORT = rospy.get_param("RPCSERVERPORT", "8000") 
        #Publisher
        self.pub1=rospy.Publisher('~recognition',String,queue_size=1000)
        #rpc server
        rospy.loginfo("Starting rpc server ...")
	#make sure the right address ip is stored in launch file, otherwise it failed
        self.server = SimpleXMLRPCServer((self.RPCSERVERIP, int(self.RPCSERVERPORT)),requestHandler=RequestHandler)
        #register server services
        self.server.register_function(self.on_word_recognized)
        rospy.loginfo("Starting rpc server ..."+self.on_word_recognized.func_name)
        rospy.loginfo('NOTIFICATION OF WORK DONE no..............')
        #create a utility
        config=rospy.get_param("CONFIG", "")
        self.ut=u.Utility(config)
        rospy.loginfo('NOTIFICATION OF WORK DONE........yes......')
        self.ut.parse()
        rospy.loginfo('NOTIFICATION OF WORK DONE........yes......')
   # notify the completion of the work by the central system(PR2)
   # status is a string integer
   # status "1" if the work was successful completed: default
   # status "-1" if the work was unsuccessful
   # return standard string integer for status successful received: "0" defaullt, -1 otherwise


   #word recognized from speech recognition

   # Register a function under a different name

   def cleanup(self):
        rospy.loginfo("Shutting down server node...")
        self.server.server_close()

   def run(self):
       while not rospy.is_shutdown():
             self.server.handle_request()
                     

   def on_word_recognized(self,x):
      if(rospy.get_param('ORDER','0')==1):
       rospy.loginfo('WORD  RECOG SERVER')
       word=self.ut.informationRetrieval(x)
       #publish word recognized
       self.pub1.publish(String(word))
       rospy.loginfo('Ergebnis is:'+word)
      return 0


if __name__=="__main__":
    
    try:
        Server().run()
    except:
        rospy.loginfo("Shutting down rpc server...")



