import time
import json
import threading
import os
import socket
import cv2
import numpy as np
from rexarm import Rexarm

class Worker:
    def __init__(self, worker_num, worker_port, master_port, master_heartbeat_port,url):
        pid = os.getpid()
        self.worker_num = worker_num
        self.worker_port = worker_port
        self.master_port = master_port
        self.master_heartbeat_port = master_heartbeat_port
        self.url = url
        self.rex = Rexarm()
		## create TCP socket on the given worker_port_number; TCP socket to receive msg; create an INET, STREAMing socket
        worker_tcp_listen = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        worker_tcp_listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        worker_tcp_listen.bind((self.url, self.worker_port))# bind the socket to the server
        worker_tcp_listen.listen(20)  # become server socket
        ## parameter
        self.IK_succeed_flag = 0
        ## setup thread
        setup_thread = threading.Thread(target=self.setup_thread)
        setup_thread.start()
        time_out_LCM_thread = threading.Thread(target=self.time_out_LCM)
        time_out_LCM_thread.start()
        time.sleep(1)

        while True:
            clientsocket, address = worker_tcp_listen.accept()
            #print "Get order from master."
            worker_job = clientsocket.recv(1024)
            worker_msg = json.loads(worker_job.decode("utf-8"))
            clientsocket.close()
            self.do_job(worker_msg)
    def time_out_LCM(self):
        while True:
            self.rex.lc.handle_timeout(50)

    def setup_thread(self):
        # setup heartbeat
        thread_send_heartbeat = threading.Thread(target=self.send_heartbeat)
    	thread_send_heartbeat.start()
    	## send ready mag to master's TCP socket
    	msg_ready = {"msg type":"status","worker num":self.worker_num,"status":"ready"}
    	msg_ready_json = json.dumps(msg_ready)
    	tcp_send_msg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    	tcp_send_msg.connect((self.url,self.master_port))
    	tcp_send_msg.sendall(str.encode(msg_ready_json))
    	tcp_send_msg.close()

    def do_job(self,worker_msg):
        # do job and get data
        task = worker_msg["task"]
        if task == "arrive":
            thread_temp = threading.Thread(target=self.arrive,args=(worker_msg["pose"],worker_msg["method"]))
            thread_temp.start()
        elif task == "waypoint":
            thread_temp = threading.Thread(target=self.waypoint,args=( worker_msg["speed"] , worker_msg["torque"]))
            thread_temp.start()
        elif task == "grab":
            thread_temp = threading.Thread(target=self.grab)
            thread_temp.start()
        elif task == "drop":
            thread_temp = threading.Thread(target=self.drop)
            thread_temp.start()
        elif task == "cmd":
            thread_temp = threading.Thread(target=self.cmd,args=([worker_msg["joint angles"]]))
            thread_temp.start()


    def send_fb2master(self,finished_task):
       # print "[Worker %s] finished job and prepare to transfer data to [Master]."%(self.worker_num)
        # send back data to master
        msg_finish = {"msg type":"status","worker num":self.worker_num,"status":"finished","task":finished_task}
        msg_finish_json = json.dumps(msg_finish)
        tcp_send_msg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_send_msg.connect((self.url,self.master_port))
        tcp_send_msg.sendall(str.encode(msg_finish_json))
        tcp_send_msg.close()


    	#print "worker_img:",self.worker_num,"ready.\n"
    def send_heartbeat(self):
    	msg_hb = {"msg type":"heartbeat","worker num":self.worker_num}
    	msg_hb_json = json.dumps(msg_hb)
        
    	while True:
    		## UDP protocol for heartbeat
            send_hb_msg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            send_hb_msg.connect((self.url,self.master_heartbeat_port))
            send_hb_msg.sendall(str.encode(msg_hb_json))
            send_hb_msg.close()
            time.sleep(3)
    def cmd(self,cmd):    ## only care anles 0~5
        self.rex.joint_angles[0]=cmd[0]
        self.rex.joint_angles[5]=cmd[5]
        self.rex.cmd_publish()
        while True:
            if abs(cmd[0]-self.rex.joint_angles_fb[0])<self.rex.base_rotate_tol:
                break

        self.rex.joint_angles[1] = cmd[1]
        self.rex.joint_angles[2] = cmd[2]
        self.rex.joint_angles[3] = cmd[3]
        self.rex.joint_angles[4] = cmd[4]
        self.rex.cmd_publish()
        while True:
            if abs(cmd[0]-self.rex.joint_angles_fb[0])<self.rex.tol_check_cmd and\
            abs(cmd[1]-self.rex.joint_angles_fb[1])<self.rex.tol_check_cmd and\
            abs(cmd[2]-self.rex.joint_angles_fb[2])<self.rex.tol_check_cmd and\
            abs(cmd[3]-self.rex.joint_angles_fb[3])<self.rex.tol_check_cmd and\
            abs(cmd[4]-self.rex.joint_angles_fb[4])<self.rex.tol_check_cmd and\
            abs(cmd[5]-self.rex.joint_angles_fb[5])<self.rex.tol_check_cmd and\
            abs(cmd[6]-self.rex.joint_angles_fb[6])<self.rex.tol_check_cmd:
                #print "self.rex.joint_angles_fb",self.rex.joint_angles_fb,"\nmotor_angle",motor_angle
                break
        self.send_fb2master("arrive")

    def arrive(self,block_pose,method):
        #input: pose(x,y,z,angle)  output:cmd to pose_LCM
        #print "Block position location: ",block_pose
        x = block_pose[0]
        y = block_pose[1]
        z = block_pose[2]
        angle = block_pose[3]
        pose = []
        self.IK_succeed_flag = 0
        ##calculate end-effector point
        ## swap method
        if method == 1:
            add_angle = [0,np.pi/2,-np.pi/2,np.pi,-np.pi,np.pi/4,-np.pi/4,np.pi/6,-np.pi/6,3*np.pi/4,-3*np.pi/4,5*np.pi/6,-5*np.pi/6,np.pi/8,-np.pi/8,7*np.pi/8,-7*np.pi/8]
            for i in range(len(add_angle)): #try four directions to find a good way
                print "Method 1 iteration time:",i
                angle_temp = angle + add_angle[i]
                pose = []
                pose.append(x- (self.rex.l5)*np.sin(angle_temp))
                pose.append(y - (self.rex.l5)*np.cos(angle_temp))
                pose.append(z + self.rex.l4_swap)
                pose.append(angle_temp)

                motor_angle = self.rex.rexarm_IK(pose,method)
                if motor_angle != 0:
                   # print "method 1 pose:",motor_angle
                    ## first calculate base angle
                    self.rex.joint_angles[0]=motor_angle[0]
                    self.rex.cmd_publish()
                    while True:
                        if abs(motor_angle[0]-self.rex.joint_angles_fb[0])<self.rex.base_rotate_tol:
                            break
                    self.rex.joint_angles[1]=motor_angle[1]
                    self.rex.joint_angles[2]=motor_angle[2]
                    self.rex.joint_angles[3]=motor_angle[3]
                    self.rex.joint_angles[4]=motor_angle[4]
                    self.rex.joint_angles[5]=motor_angle[5]
                    self.rex.cmd_publish()
                    self.IK_succeed_flag = 1
                    break
        elif method == 2:

            #pose.append(x- (self.rex.l4_swap)*np.sin(angle))
            #pose.append(y- (self.rex.l4_swap)*np.cos(angle))
            #pose.append(z + self.rex.l5)
            pose.append(x- (self.rex.l5)*np.sin(angle))
            pose.append(y- (self.rex.l5)*np.cos(angle))
            pose.append(z - self.rex.l4_swap)
            pose.append(angle)
            motor_angle = self.rex.rexarm_IK(pose,method)
            if motor_angle != 0:
               # print "method 2 pose:",motor_angle
                ## first calculate base angle
                self.rex.joint_angles[0]=motor_angle[0]
                self.rex.cmd_publish()
                while True:
                    #print "Turn base.",motor_angle[0],self.rex.joint_angles_fb[0]
                    if abs(motor_angle[0]-self.rex.joint_angles_fb[0])<self.rex.base_rotate_tol:
                        break
                self.rex.joint_angles[5]=motor_angle[5]
                self.rex.cmd_publish()
                while True:
                    #print "Turn base.",motor_angle[0],self.rex.joint_angles_fb[0]
                    if abs(motor_angle[5]-self.rex.joint_angles_fb[5])<self.rex.base_rotate_tol:
                        break
                self.rex.joint_angles[1]=motor_angle[1]
                self.rex.joint_angles[2]=motor_angle[2]
                self.rex.joint_angles[3]=motor_angle[3]
                self.rex.joint_angles[4]=motor_angle[4]
                self.rex.joint_angles[5]=motor_angle[5]
                self.rex.cmd_publish()
                self.IK_succeed_flag = 1

        elif method == 3:
            # this is different from method 1 and 2. the input pose to IK is block position coordinate (x,y,z,theta),theta is loop angle 
            theta = 0
            pose.append(x)
            pose.append(y)
            pose.append(z+12)
            pose.append(theta)
            speed = 0.6

            try: 
                speed = block_pose[5] 
                self.rex.max_torque =  block_pose[6]               
                print "speed:",speed
            except:
                pass
            self.rex.speed_list[1] = speed
            self.rex.speed_list[2] = speed
            self.rex.speed_list[3] = speed
            #self.rex.speed_list[6] = 0.4
            self.rex.cmd_publish()
            try:
                wrist_angle_cond = block_pose[4]
            except:
                print "wrist_angle is by default."
                wrist_angle_cond = 0
            pose.append(wrist_angle_cond)#usually it is 0, event 3 4 final point pi/2 

            motor_angle = 0
            sign = 1
            count = 0
            delta = 0.02
            delta_sum = 0
            while not motor_angle:
                count+=1
                motor_angle = self.rex.rexarm_IK(pose,method)
                if count % 2 == 1:
                    delta_sum += delta
                pose[3] = delta_sum*sign
                sign = -sign
                if count >50000:
                    break
            if motor_angle != 0:
                ## first calculate base angle
                self.rex.joint_angles[0]=motor_angle[0]
                self.rex.joint_angles[5]=motor_angle[5]
                self.rex.cmd_publish()
                while True:
                    #print "Turn base.",motor_angle[0],self.rex.joint_angles_fb[0]
                    if abs(motor_angle[0]-self.rex.joint_angles_fb[0])<self.rex.base_rotate_tol\
                    and abs(motor_angle[5]-self.rex.joint_angles_fb[5])<self.rex.wrist_rotate_tol:
                        break
                self.rex.joint_angles[1]=motor_angle[1]
                self.rex.joint_angles[2]=motor_angle[2]
                self.rex.joint_angles[3]=motor_angle[3]
                self.rex.joint_angles[4]=motor_angle[4]
                self.rex.cmd_publish()
                self.IK_succeed_flag = 1

        if self.IK_succeed_flag == 1:
            while True:
                if abs(motor_angle[0]-self.rex.joint_angles_fb[0])<self.rex.tol_check and\
                abs(motor_angle[1]-self.rex.joint_angles_fb[1])<self.rex.tol_check and\
                abs(motor_angle[2]-self.rex.joint_angles_fb[2])<self.rex.tol_check and\
                abs(motor_angle[3]-self.rex.joint_angles_fb[3])<self.rex.tol_check and\
                abs(motor_angle[4]-self.rex.joint_angles_fb[4])<self.rex.tol_check and\
                abs(motor_angle[5]-self.rex.joint_angles_fb[5])<self.rex.tol_check:
                    #print "self.rex.joint_angles_fb",self.rex.joint_angles_fb,"\nmotor_angle",motor_angle
                    print "Break arrive." 
                    break
            self.send_fb2master("arrive")
        else:
            print "-------------------IK failed.----------------------"
            self.send_fb2master("failed")


    def waypoint(self,speed,torque):
        try: 
            self.rex.speed_list[1] = speed
            self.rex.speed_list[2] = speed
            self.rex.speed_list[3] = speed
            self.rex.max_torque =torque
        except:
            pass
        self.rex.joint_angles[1] = 0
        self.rex.joint_angles[2] = 0
        self.rex.joint_angles[3] = 0
        #self.rex.max_torque=1
        self.rex.cmd_publish()
        while True:
            if abs(self.rex.joint_angles_fb[1])<self.rex.tol_check and\
            abs(self.rex.joint_angles_fb[2])<self.rex.tol_check and\
            abs(self.rex.joint_angles_fb[3])<self.rex.tol_check:
                print "Break waypoint." 
                #print "self.rex.joint_angles_fb",self.rex.joint_angles_fb,"\nmotor_angle",motor_angle
                break
        self.send_fb2master("arrive")

    def grab(self):
        # Gripper 
        #self.rex.speed_list[6] = 0.4
        self.rex.max_torque = 0.6##
        self.rex.joint_angles[6] = self.rex.grab_angle
        self.rex.cmd_publish()
        #aaaa_thread = threading.Thread(target=self.aaaa)
        #aaaa_thread.start()
        while True:
            if  self.rex.load_fb[6] > self.rex.grab_load:# and abs(self.rex.grab_angle-self.rex.joint_angles_fb[6])<self.rex.angle_grab_tol:#self.rex.load_fb[6] > self.rex.grab_load and
                print "Break grab." 
                break
        time.sleep(0.3)
        self.send_fb2master("grab")

    def drop(self):
        #self.rex.speed_list[6] = 0.3
        self.rex.max_torque = 0.4##
        self.rex.joint_angles[6] = self.rex.drop_angle
        self.rex.cmd_publish()
        while True:
            if abs(self.rex.drop_angle-self.rex.joint_angles_fb[6])<self.rex.angle_drop_tol:#self.rex.load_fb[6] < self.rex.drop_load and 
                print "Break drop." 
                break
        time.sleep(0.4)
        self.send_fb2master("drop")
        
    def aaaa(self):
        while True:
            print "self.rex.joint_angles_fb",self.rex.joint_angles_fb
            time.sleep(0.5)
