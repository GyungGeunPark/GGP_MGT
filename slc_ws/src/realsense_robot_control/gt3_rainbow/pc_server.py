import pickle as plk
import numpy as np
import sys
import rclpy
import threading
import time

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg as std_msgs

from datetime import datetime
from multiprocessing import Process, Pipe, Queue
from PyQt5.QtWidgets import *
from PyQt5 import uic
from gt3_rainbow import main


class cmd_Subscriber(Node):

    def __init__(self, t0, t1, t2, b_pc):
        super().__init__('gtm_rcv')
        
        self.t0 = t0
        self.t1 = t1
        self.t2 = t2
        self.pc_pipe = b_pc
        
        self.sub_gtm = self.create_subscription(std_msgs.String, '/mobile_to_robot', self.cmd_Callback, 10)
    
    
    def cmd_Callback(self, msg):
        received_data = msg.data.split(';')
        print(" > RCM cmd : ", received_data)
        if (received_data[0] == "ROBOT"):
            pass
    
        elif (len(received_data) != 0):
            if  received_data[0] == "paint" :
                self.t0.put(received_data[0])
              
            elif received_data[0] == "detect":
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'front':
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'rear':
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'tower':
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'arm':
                self.t0.put(received_data[0])                
                
            elif received_data[0] == 'return':
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'connect':
                self.t0.put(received_data[0])
                
            elif received_data[0] == 'home':
                self.t0.put(received_data[0])
      
            elif received_data[0] == 'pose':
                self.t0.put(received_data)      
                  
            elif received_data[0] == 'plane':
                self.t0.put(received_data[0])   
                
            elif received_data[0] == 'curved':
                self.t0.put(received_data[0])  
                
            elif received_data[0] == 'topview':
                self.t0.put(received_data[0])   
                
            elif received_data[0] == 'floorview':
                self.t0.put(received_data[0])       
                
            elif received_data[0] == 'right_cam':
                self.t0.put(received_data[0])   
                
            elif received_data[0] == 'arm_tilt':
                self.t0.put(received_data[0]) 
                
            elif received_data[0] == 'arm_camera_left':
                self.t0.put(received_data[0])       
            
            elif received_data[0] == 'leftwall':
                self.t0.put(received_data[0])  
                
            elif received_data[0] == 'leftcorner':
                self.t0.put(received_data[0])  
                
            elif received_data[0] == 'frontwall':
                self.t0.put(received_data[0])  
            
            elif received_data[0] == 'heart':
                self.t0.put(received_data[0])     
            
            elif received_data[0] == 'refresh':
                self.t0.put(received_data[0]) 
                
            elif received_data[0] == 'pc_refresh':
                self.t0.put(received_data[0])        
            
            elif received_data[0] == 'cam_restart':
                self.t1.put('quit')
            elif received_data[0] == 'rainbow_start':
                self.t2.put('start')    
                                       
        else:
            pass    
        

class Device():
    def __init__(self, pipe, t1, r, camera):
        self.device = dSensor(pipe, t1, r, camera)
        
        l515 = [(640, 480, 30), (320, 240, 30)]
        d455 = [(1280, 720, 30), (640, 480, 30)]
        #if camera == 'arm':
        #    d455 = [(640, 480, 30), (640, 480, 30)]
        
        self.device.cam_setting(d455)
        self.device.run()
        
    
def sub_loop(t0, t1, t2, b_pc):
    rclpy.init()
    pcs = cmd_Subscriber(t0, t1, t2, b_pc)
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(pcs)
    
    try:
        executor.spin()
    except:
        pcs.destroy_node()
        rclpy.shutdown()  
        print('###ros2 shutdown###')
        
def connect_rainbow():
    #while True:
    #    if t2.qsize() != 0:
    #        get = t2.get()
    #        print(get)
    #        if get == 'start':
    #            break
    #    time.sleep(1)
                
    rclpy.init()
    ps = main.pose_Subscriber()
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(ps)
    try:
        executor.spin()
    except:
        ps.destroy_node()
        rclpy.shutdown()  
        print('###rainbow ros2 shutdown###')     
    
def manager():
    while True:
        m1 = Process(target=Source, args=(a_pipe, t1, r3, 'arm'))     
        m1.start()
        m2 = Process(target=pc_process, args=(b_pipe, t0, b_pc, r0, r1, r2, r3, 'arm'))   
        m2.start()
        m1.join()
        m2.terminate()
        
        time.sleep(3)      
        print('restart')  

def Source(pipe, t1, r, camera):
    source = Device(pipe, t1, r, camera)


def tcp_server(HOST, PORT, a_pc):
    server = Socket(HOST, PORT, a_pc)


if __name__ == '__main__':
    
    a_pipe, b_pipe = Pipe()                # nodeD455 - pc_process
    a_pc, b_pc = Pipe(duplex=True)         # socket_server - pc_process
    
    t0 = Queue()      # pc_server - pc_process  
    t1 = Queue()      # pc_server - node455  
    t2 = Queue()      # pc_server - rainbow
    
    r0 = Queue()      # pc_process - node455   :   request frame front
    r1 = Queue()      # pc_process - node455   :   request frame rear
    r2 = Queue()      # pc_process - node455   :   request frame tower
    r3 = Queue()      # pc_process - node455   :   request frame arm
    
    
    s = Process(target=sub_loop, args=(t0, t1, t2, b_pc))
    s.start()
    
    r = Process(target=connect_rainbow, args=())
    r.start()
    

