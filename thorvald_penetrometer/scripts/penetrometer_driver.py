#!/usr/bin/env python

import rospy
import actionlib
import serial
import threading

#import actionlib_msgs.msg
import dynamic_reconfigure.server # import Server
from thorvald_penetrometer.cfg import PenetrometerConfig


import thorvald_penetrometer.msg

class PenetrometerServer(object):
    """
     Class for Penetrometer Control
    
    """
    _feedback = thorvald_penetrometer.msg.ProbeSoilFeedback()
    _result   = thorvald_penetrometer.msg.ProbeSoilResult()


    def __init__(self, name):
        """
         Initialization for Class
        
        """
        self.cancelled = False
        self.running=True
        self.reply_buf=[]
        
        self.serial_port = rospy.get_param('~serial_port', '/dev/tnt0')     
        
        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(name, thorvald_penetrometer.msg.ProbeSoilAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        #Creating Dyn reconf server
        rospy.loginfo("Creating dynamic reconfigure server.")
        self.dynsrv = dynamic_reconfigure.server.Server(PenetrometerConfig, self.dyn_reconf_callback)
        

        rospy.loginfo("ROS init done ...")
        
        rospy.loginfo("initialising device done ...")
        
        rospy.loginfo("opening serial port")
        self.ser = serial.Serial(self.serial_port, 57600, timeout=0, parity=serial.PARITY_NONE)
        
        thread = threading.Thread(target=self.read_from_port)#, args=(serial_port,))
        thread.start()
        
        
        rospy.loginfo("ALL done ...")
        
        self.initialise_penetrometer()
        rospy.spin()
        
        self.running=False
        self.ser.close()
        
    def read_from_port(self):
        serial_buffer=[]
        #response=[]
        while self.running:
            if (self.ser.inWaiting()>0):                                        #if incoming bytes are waiting to be read from the serial input buffer
                data_str = self.ser.read(self.ser.inWaiting())#.decode('ascii')  #read the bytes and convert from binary array to ASCII
                for i in data_str:
                    serial_buffer.append(i)
                if '\n' in serial_buffer:
                    print "end found"
                    nind= serial_buffer.index('\n')
                    self.reply_buf.append(serial_buffer[0:nind])
                    for i in reversed(range(nind+1)):
                        serial_buffer.pop(i)
                    print serial_buffer

                if len(self.reply_buf)>0:
                    print(self.reply_buf)      
        rospy.sleep(0.1) # Optional: sleep 10 ms (0.01 sec) once per loop to let other threads on your PC run 

    def dyn_reconf_callback(self, config, level):
        #rospy.loginfo("""Reconfigure Request: {counts}""".format(**config))
        #self.max_counts = config['counts']
        print "reconfigure ", config
        return config

    def send_command(self, command):
        for i in command:
            self.ser.write(i)
            rospy.sleep(0.001)
        self.ser.write('\n')
    
    def wait_for_reply(self, timeout=10):
        time_count=0
        response=''
        replied=False
        while not replied and time_count<= (timeout*20) :
            if len(self.reply_buf)>0:
                response = self.reply_buf.pop(0)
                replied = True
            else:
                rospy.sleep(0.05)
                time_count+=1
                
        return response
    
    def initialise_penetrometer(self, retries=3):
        self.send_command('@')
        rospy.loginfo("waiting for initialisation confirmation")
        response = self.wait_for_reply()
        if ''.join(response) == "@1":
            rospy.loginfo("initialisation correct!")
        else:
            if retries > 0:
                rospy.loginfo("wrong response try again")
                self.initialise_penetrometer(retries=retries-1)
            else:
                rospy.loginfo("too many fails!!")
#        self.wait_for_reply


    def executeCallback(self, goal):
        self.cancelled=False
        count = 0
        while count<10 and not self.cancelled:
            rospy.sleep(1.0)
            rospy.loginfo('%s' % count)
            self._feedback.state=str(count)
            self._as.publish_feedback(self._feedback)
            count+=1
        
        if not self.cancelled:
            self._result.result = True
            rospy.loginfo('%s: Succeeded')
            self._as.set_succeeded(self._result)
        else:
            self._as.set_preempted()
    
    def preemptCallback(self):
        self.cancelled=True
    

if __name__ == '__main__':
    rospy.init_node('thorvald_penetrometer')
    server = PenetrometerServer(rospy.get_name())
