#!/usr/bin/env python

import rospy
import actionlib
import serial
import threading
import yaml

import std_srvs.srv
#import actionlib_msgs.msg
import dynamic_reconfigure.server # import Server
from thorvald_penetrometer.cfg import PenetrometerConfig

import std_msgs.msg #import String
import thorvald_penetrometer.msg



class PenetrometerServer(object):
    """
     Class for Penetrometer Control
    
    """
    _feedback = thorvald_penetrometer.msg.ProbeSoilFeedback()
    _result   = thorvald_penetrometer.msg.ProbeSoilResult()

    _config_commands={ "ground_level":'g', "speed":'s', "top_speed":'t', "home_speed":'k',
                    "acceleration":'a', "deceleration":'d', "probe_depth":'l', "dist_per_reading": 'q',
                    "lfd_tolerance":'r', "max_force":'m', "min_force":'n', "max_force_delta":'x',
                    "min_force_delta":'y',"force_delta_abs":'v',"safe_disconnect":'o'}
    _decimilimiters_configs=['ground_level','probe_depth','dist_per_reading']
    _centiseconds_configs=['lfd_tolerance']



    def __init__(self, name):
        """
         Initialization for Class
        
        """
        self.cancelled = False
        self.running=True
        self.reply_buf=[]
        self.last_response = ''
        self.config={}
        
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')     

        rospy.loginfo("opening serial port")
        self.ser = serial.Serial(self.serial_port, 57600, timeout=0, parity=serial.PARITY_NONE)
        thread = threading.Thread(target=self.read_from_port)#, args=(serial_port,))
        thread.start()
        
        self.input_pub = rospy.Publisher('/penetrometer_raw_data', std_msgs.msg.String, queue_size=0)
        
        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(name, thorvald_penetrometer.msg.ProbeSoilAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")


        rospy.loginfo("Creating services")
        sname=name+'/save_params'
        s = rospy.Service(sname, std_srvs.srv.Trigger, self.save_params)
        sname=name+'/clear_errors'
        s1 = rospy.Service(sname, std_srvs.srv.Trigger, self.clear_errors_req)


        rospy.loginfo("initialising device done ...")
        self.initialise_penetrometer()
        
        self.clear_errors()
        self.set_e_stop(False)
        self.set_power_enable(True)


        #Creating Dyn reconf server
        rospy.loginfo("Creating dynamic reconfigure server.")
        self.dynsrv = dynamic_reconfigure.server.Server(PenetrometerConfig, self.dyn_reconf_callback)
        
        self.send_home()
        rospy.loginfo("ROS init done ...")
        rospy.loginfo("ALL done ...")
        
        rospy.spin()
        
        #self.write_config_to_file()
        self.running=False
        self.ser.close()
        
    
    
    def clear_errors_req(self, req):
        self.clear_errors()
        return True, "Errors cleared"
        
    def save_params(self, req):
        self.write_config_to_file()
        return True, "saved to params.yaml"        
        
    
        
    def write_config_to_file(self):
        config = dict(self.config)
        del config['groups']
        
        yml = yaml.safe_dump(config, default_flow_style=False)
        
        fh = open("params.yaml", "w")
        s_output = str(yml)
        fh.write(s_output)
        fh.close()



    def read_from_port(self):
        serial_buffer=[]
        #response=[]
        while self.running:
            if (self.ser.inWaiting()>0):                                        #if incoming bytes are waiting to be read from the serial input buffer
                data_str = self.ser.read(self.ser.inWaiting())#.decode('ascii')  #read the bytes and convert from binary array to ASCII
                print "|"+data_str+"|"
                for i in data_str:
                    serial_buffer.append(i)
                while '\n' in serial_buffer:
                    #print "end found"
                    nind= serial_buffer.index('\n')
                    self.reply_buf.append(serial_buffer[0:nind])
                    pen_data = ''.join(serial_buffer[0:nind])
                    self.input_pub.publish(pen_data)
                    for i in reversed(range(nind+1)):
                        serial_buffer.pop(i)
                    print serial_buffer


#                if len(self.reply_buf)>0:
#                    print(self.reply_buf)      
            rospy.sleep(0.001) # Optional: sleep 10 ms (0.01 sec) once per loop to let other threads on your PC run 


    def dyn_reconf_callback(self, config, level):
        #rospy.loginfo("""Reconfigure Request: {counts}""".format(**config))
        #self.max_counts = config['counts']
        #print "reconfigure ", config
        if self.config:
            changed_dict = {x: self.config[x] != config[x] for x in self.config if x in config}
            lk = [key  for (key, value) in changed_dict.items() if value]
            #print "config changed ", lk, config[lk[0]]
            self.set_config(lk[0], config[lk[0]])
            self.config = config
        else:
            #print "First config: ", config.items()            
            self.config = config
            for i in config.items():
                self.set_config(i[0], i[1])
                rospy.sleep(0.1)
        return config


    def set_config(self, field, value):
        if field in self._config_commands.keys():
            print field, value
            if isinstance(value,bool):
                value=int(value)
            if field in self._decimilimiters_configs:
                value=int(value*10)
            if field in self._centiseconds_configs:
                value=int(value/10)
            command = self._config_commands[field]+str(value)
            reply = self._config_commands[field].capitalize()+str(value)
            print command, reply
            self.send_command(command)
            response = self.wait_for_reply(reply)
            if response:
                rospy.loginfo("%s set at %d" %(field, value))
            else:
                rospy.logerr("Something failed when setting %s set at %d, response code (%s)" %(field,value,self.last_response))
                rospy.loginfo("Maybe, try again?")

    def send_command(self, command):
        for i in command:
            self.ser.write(i)
            rospy.sleep(0.001)
        self.ser.write('\n')
    
    
    def clear_reply_buf(self):
        self.reply_buf=[]
    
    
    def wait_for_reply(self, expected, timeout=10):
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
        
        self.last_response = ''.join(response)
        if self.last_response == expected:
            return True
        else:
            if time_count > (timeout*20):
                self.last_response = 'timeout'
            return False
    


    def initialise_penetrometer(self, retries=3):
        self.clear_reply_buf()
        self.send_command('@')
        rospy.loginfo("waiting for initialisation confirmation")
        response = self.wait_for_reply("@1")
        if response:
            rospy.loginfo("initialisation correct!")
        else:
            if retries > 0:
                rospy.logwarn("wrong response try again")
                self.initialise_penetrometer(retries=retries-1)
            else:
                rospy.logerr("too many fails!!")

    def clear_errors(self):
        self.clear_reply_buf()
        self.send_command('f0')
        rospy.loginfo("clearing errors")
        response = self.wait_for_reply('F0 ')
        if response:
            rospy.loginfo("Errors cleared")
        else:
            rospy.logerr("Something failed, response code (%s)" %self.last_response)
            rospy.loginfo("Maybe, try clear error service again?")


    def set_e_stop(self, enable):
        self.clear_reply_buf()
        if enable:
            command = 'e1'
            reply = 'E1'
        else:
            command = 'e0'
            reply = 'E0'
        self.send_command(command)
        
        rospy.loginfo("clearing estop")
        response = self.wait_for_reply(reply)
        if response:
            rospy.loginfo("estop reset")
        else:
            rospy.logerr("Something failed, response code (%s)" %self.last_response)
            rospy.loginfo("Maybe, try set estop service again?")

        
    def set_power_enable(self, enable):
        self.clear_reply_buf()
        if enable:
            command = 'p1'
            reply = 'P1'
        else:
            command = 'p0'
            reply = 'P0'
        self.send_command(command)
        
        rospy.loginfo("Enabling")
        response = self.wait_for_reply(reply)
        if response:
            rospy.loginfo("Enabled")
        else:
            rospy.logerr("Something failed, response code (%s)" %self.last_response)
            rospy.loginfo("Maybe, try set power service again?")
        

    def send_home(self):
        self.clear_reply_buf()
        self.send_command('z')
        rospy.loginfo("Homing")
        response = self.wait_for_reply('Z1', timeout=120)
        if response:
            rospy.loginfo("Homed")
        else:
            rospy.logerr("Something failed, response code (%s)" %self.last_response)
            rospy.loginfo("Maybe, try send home service again?")        



    def get_data(self, timeout=10):
        print "."
        time_count=0
        response=''
        replied=False
        while (not replied) and time_count <= (timeout*20) :
            #print (self.reply_buf)
            if len(self.reply_buf)>0:
                response = self.reply_buf.pop(0)
                response = ''.join(response)
                #print "data in ->", response
                replied = True
            else:
                rospy.sleep(0.05)
                time_count+=1

        if time_count > (timeout*20):
            #print "timed out"
            return None
        else:
            return response

    def executeCallback(self, goal):
        self.cancelled=False
        finished=False
        self.depth_data=[]
        self.force_data=[]

        self.clear_reply_buf()
        rospy.loginfo("Probing")
        self.send_command('!1')
        while not self.cancelled and not finished:
            print "++"
            data_str = self.get_data(timeout=15)
            print data_str
            
            if data_str:
                if data_str == '!1':
                    finished=True
                else:
                    print "appending"
                    cd=data_str.lstrip('*').split(',')
                    self.depth_data.append(int(cd[0]))
                    self.force_data.append(int(cd[1]))
            else:
                self.cancelled=True
        
        rospy.loginfo("Probe finished")
        self.send_home()
        
        if not self.cancelled:
            self._result.result = True
            self._result.depth = self.depth_data
            self._result.force = self.force_data
            rospy.loginfo('Succeeded')
            self._as.set_succeeded(self._result)
        else:
            self._as.set_preempted()
    
    def preemptCallback(self):
        self.cancelled=True
    

if __name__ == '__main__':
    rospy.init_node('thorvald_penetrometer')
    server = PenetrometerServer(rospy.get_name())
