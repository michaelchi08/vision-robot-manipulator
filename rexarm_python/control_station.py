import sys
import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
from ui import Ui_MainWindow
from rexarm import Rexarm
import os,signal
from video import Video
import time
import threading
import socket
import json
from multiprocessing import Process 
import worker
import freenect
from imgProcess import ImgProcess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

""" Pyxel Positions of image in GUI """
MIN_X = 310
MAX_X = 950

MIN_Y = 30
MAX_Y = 510

BLOCK_HIGH = 38
Z_DEPTH_COMPENSATION = 0 #for both swap and pick
METHOD_DISTANCE_LINE_INNER = 175# decide which method we use: 1 or 2 i.e. swap or pick
METHOD_DISTANCE_LINE_OUTER = 255
# parameters from lab
DEPTH_RATIO = 2

class Gui(QtGui.QMainWindow):
    """ 
    Main GUI Class
    It contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self,parent=None):
        # some pre-definition
        self.color_name = ["black","red","orange","yellow","green","blue","violet","pink"]
        self.teach_repeat_count = 0
        self.record_B = []
        self.record_S = []
        self.record_E = []
        self.record_W = []
        self.teach_repeat_hole_num = 2
        self.index_repeat = 0
        self.start_teach_repeat_in_play = 0
        self.end_effector = [] # for link i
        self.video_mark = 1
        # gripper safe distance
        self.dist_gripper = 30
        self.dist_z_lift = 30
        self.task_one_step_unfinished = 1
        ## stable_points_data_mark  1st args: 1 or   2ed args: priority
        self.points_data_mark = [0,0,0,0,0,0,0,0]# used for event
        self.points_data = [[],[],[],[],[],[],[],[]]
        ##############################################################
        ## for task 3,4,5 middle point 
        self.transition_point_3 = [230,-115,19,0]
        self.transition_point = [150,-180,19,0]
        self.transition_point_5 = [200,-80,19,0]

        ##############################################################
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
                
        """ Main Variables Using Other Classes"""
        self.rex = Rexarm()
        self.video = Video()
        """ Other Variables """
        self.last_click = np.float32([0,0])

        """ Set GUI to track mouse """
        QtGui.QWidget.setMouseTracking(self,True)

        """ 
        Video Function 
        Creates a timer and calls play() function 
        according to the given time delay (27ms) 
        """
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.play)
        self._timer.start(27)
       
        """ 
        LCM Arm Feedback
        Creates a timer to call LCM handler continuously
        No delay implemented. Reads all time 
        """  
        self._timer2 = QtCore.QTimer(self)
        self._timer2.timeout.connect(self.rex.get_feedback)
        self._timer2.start()

        """ 
        Connect Sliders to Function
        LAB TASK: CONNECT THE OTHER 5 SLIDERS IMPLEMENTED IN THE GUI 
        """ 
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip2.valueChanged.connect(self.sliderChange)

        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
        
        #tolerance if end effector reach certain point
        self.tol_check = 0.1 # Radians
        # load calibration file (RGB pixel coordinates to world coordinates)
        try:
            self.aff_matrix_diy = np.loadtxt(open("./calibration/calibration_matrix.csv"),delimiter = ",")
        except:
            print "Calibration file does not exist, calibrate first."
        """ Commands the arm as the arm initialize to 0,0,0,0 angles """
        self.sliderChange() 
        
        """ Connect Buttons to Functions 
        LAB TASK: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btnUser1.setText("Affine Calibration")
        self.ui.btnUser1.clicked.connect(self.affine_cal)
        # Task: teach and repeat

        self.ui.btnUser2.setText("Clear_recorded")
        self.ui.btnUser2.clicked.connect(self.teach_repeat_clear)

        self.ui.btnUser3.setText("Teach")
        self.ui.btnUser3.clicked.connect(self.teach_repeat_record)

        self.ui.btnUser4.setText("Repeat")
        self.ui.btnUser4.clicked.connect(self.teach_repeat_publish_initial)



        self.ui.btnUser5.setText("Repeat (Cubic spline)")
        self.ui.btnUser5.clicked.connect(self.teach_repeat_publish_cubic_initial)

        self.ui.btnUser6.setText("draw")
        self.ui.btnUser6.clicked.connect(self.draw)
        #self.ui.btnUser5.setText("Click get")
       # self.ui.btnUser5.clicked.connect(self.click_get)

        self.ui.btnUser8.setText("event_1")
        self.ui.btnUser8.clicked.connect(self.event_1)

        self.ui.btnUser9.setText("event_2")
        self.ui.btnUser9.clicked.connect(self.event_2)

        self.ui.btnUser10.setText("event_3")
        self.ui.btnUser10.clicked.connect(self.event_3)

        self.ui.btnUser11.setText("event_4")
        self.ui.btnUser11.clicked.connect(self.event_4)

        self.ui.btnUser12.setText("event_5")
        self.ui.btnUser12.clicked.connect(self.event_5)

        self.img_proc_dyn = ImgProcess()
        #self.multi_proc_init()

        self.job_finished = 0
        self.teach_repeat_clear()

    def det_anles(self):
        print "self.rex.joint_angles_fb:",self.rex.joint_angles_fb

    def multi_proc_init(self):
        print "Initialize multi processes and multi threads."
        ## worker definition
        self.img_fb_data = {}
        self.img_cur = []
        self.worker_num = 4 # how many worker_img processes you want
        self.master_port = 3200
        self.url = "127.0.1.9"
        ## create TCP socket on the given master_port_number; TCP socket to receive msg; create an INET, STREAMing socket
        self.master_tcp_listen = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.master_tcp_listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        temp_addr_mark=1;
        while temp_addr_mark:
            try:
                self.master_tcp_listen.bind((self.url, self.master_port))# bind the socket to the server
                temp_addr_mark = 0
            except:
                self.master_port += 2
        self.master_tcp_listen.listen(20)  # become server socket
        self.master_heartbeat_port = self.master_port - 1000
        self.worker_port_number = self.master_port + 2000
        self.worker_list = [] # track all worker processes info
        self.worker_processes = [] # save all worker_img processes objects
        print "                 [Master] port is ", self.master_port, \
        ", \n                 [Worker] ports are:", [(self.worker_port_number+x) for x in range(self.worker_num)],\
        ", \n                 [Heartbeat] port is: ", self.master_heartbeat_port
        
        ## start a new thread set_up_worker() to set up  worker_img_processing & worker_mat_cal
        self.thread_set_up_worker = threading.Thread(target=self.set_up_worker)
        self.thread_set_up_worker.start()

    def worker_job(self,worker_task):
        #print "[Master] send job."
        task_received_flag = True
        while task_received_flag:
            for i in range(self.worker_num):
                if self.worker_list[i]["status"] == "ready":
                    worker_task_json = json.dumps(worker_task)
                    worker_send_msg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    worker_send_msg.connect((self.url,self.worker_list[i]["worker port"]))
                    worker_send_msg.sendall(str.encode(worker_task_json))
                    worker_send_msg.close()
                    self.worker_list[i]["status"] = "busy"
                    print "[Worker %s] is busy."%(i)
                    task_received_flag = False
                    break
        
    def set_up_worker(self):
        #print "set_up_worker thread: ", threading.current_thread()
        print "[Master] Setup threads."
        self.thread_listen_job = threading.Thread(target=self.listen_job)
        self.thread_listen_job.start()
        for worker_num in range(self.worker_num):
            #print worker_num
            worker_port = self.worker_port_number + worker_num
            p = Process(target=worker.Worker,
                args=(worker_num, worker_port, self.master_port, self.master_heartbeat_port,self.url))
            self.worker_processes.append(p)
            p.start()
            pid = p.pid
            self.worker_list.append({'worker num':worker_num,'status':"created",'job':[],"worker port":worker_port})
            print "[Worker %s] is created."%(worker_num)
        # create a new thread to listen for heartbeat messages
        thread_listen_heartbeat = threading.Thread(target=self.listen_heartbeat)
        thread_listen_heartbeat.start()

    def listen_job(self):
        #print "[Master] listen_job thread."
        while True:
            clientsocket,address = self.master_tcp_listen.accept()
            message = clientsocket.recv(1024)
            clientsocket.close()
            msg = json.loads(message.decode("utf-8"))#msg = json.loads(message.decode("utf-8"))
            if msg["msg type"] == "status":
                temp_tcp_status = msg["status"]
                if temp_tcp_status == "ready":
                    self.worker_list[msg["worker num"]]["status"] = "ready"
                    print "[Worker %s] is ready."%(msg["worker num"])
                elif temp_tcp_status == "finished":
                    self.worker_list[msg["worker num"]]["status"] = "ready"
                    print "[Worker %s]'s job [%s] finished and is ready."%(msg["worker num"],msg["task"])
                    #self.img_fb_data = msg["task"]
                    if msg["task"] == "failed":
                        self.job_finished = -1
                        print "job_finished = -1"
                    else:
                        self.job_finished = 1
                        print "job_finished = 1"

    def listen_heartbeat(self):
        print "---------------------------------------------------\n"
        #print "listen_heartbeat thread: ", threading.current_thread()
        print "[Master] listen_heartbeat thread."
        ## UDP protocol for heartbeat
        socket_hb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)# UDP
        socket_hb.bind((self.url, self.master_heartbeat_port))
        
        while True:
            #print "os process:",os.getpid(),", worker process:", \
            #str(self.worker_img_processes[i].pid for i in range(len(self.worker_img_processes)))
            #print "thread number:",threading.active_count(),"thread list:",threading.enumerate()
            message, address = socket_hb.recvfrom(1024)
            message = json.loads(message)
          #  print "                               \
           # [Worker %s]'s %s received."%(message["worker num"], message["msg type"])
        
    def shutdown_p(self):
        try:
            for i in range(len(self.worker_processes)):
                self.worker_processes[i].terminate()
        except:
            pass
        os.kill(os.getpid(),signal.SIGTERM)
        #print "shutdown:\n" ,"os process:",os.getpid(),", worker process:", str(self.worker_img_processes[i].pid for i in range(len(self.worker_img_processes)))

    def switch_video(self):
        try:
            block_loc = self.img_fb_data
            frame = self.img_cur
            for color_index in range(8):
                if block_loc[self.color_name[color_index]] != []:
                    cX = block_loc[self.color_name[color_index]][0]
                    cY = block_loc[self.color_name[color_index]][1]
                    cZ = block_loc[self.color_name[color_index]][2]
                    cv2.circle(frame,(cX,cY),2,(0,0,0),-1)
                    cv2.putText(frame,"%s Center:%d,%d,%d"%(self.color_name[color_index],cX,cY,cZ),(cX-20,cY-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)

            img=QtGui.QImage(frame,
                         frame.shape[1],
                         frame.shape[0],
                         QtGui.QImage.Format_RGB888
                         )
            self.ui.videoFrame.setPixmap(QtGui.QPixmap.fromImage(img))
        except:
            pass
    def switch_video_dyn(self):
        frame = freenect.sync_get_video()[0]
        frame_d = freenect.sync_get_depth()[0]
        block_loc = self.img_proc_dyn.get_all_coord(frame,frame_d)
        #print block_loc
        try:
            for color_index in range(8):
                if block_loc[self.color_name[color_index]] != []:
                    cX = block_loc[self.color_name[color_index]][0]
                    cY = block_loc[self.color_name[color_index]][1]
                    cZ = block_loc[self.color_name[color_index]][2]
                    angle = block_loc[self.color_name[color_index]][3]
                    level = block_loc[self.color_name[color_index]][4]
                    world_coord_temp = np.dot(self.aff_matrix_diy,[cX,cY,1])
                    dist = np.sqrt(world_coord_temp[0]**2+world_coord_temp[1]**2)
                    cv2.circle(frame,(cX,cY),2,(0,0,0),-1)
                    cv2.putText(frame,"%s:%d,%d,%d,%d,%d"%(self.color_name[color_index],\
                        int(world_coord_temp[0]),int(world_coord_temp[1]),int(dist),cZ,level),(cX-20,cY-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
            img=QtGui.QImage(frame,
                         frame.shape[1],
                         frame.shape[0],
                         QtGui.QImage.Format_RGB888
                         )
            self.ui.videoFrame.setPixmap(QtGui.QPixmap.fromImage(img))
        except:
            pass
 
    def click_get(self):
        print "self.rex.joint_angles_fb",self.rex.joint_angles_fb

    #### get initial image info 
    def get_point_from_img_method3(self,limit):
        if limit == 1:
            y_limit = 0
        elif limit == 2:
            y_limit =-140
        elif limit == 3:
            y_limit = -150
        elif limit ==4:
            y_limit =-310

        self.points_data = [[],[],[],[],[],[],[],[]]
        frame = freenect.sync_get_video()[0]
        frame_d = freenect.sync_get_depth()[0]
        block_loc = self.img_proc_dyn.get_all_coord(frame,frame_d)
        #print "block_loc:",block_loc
        for color_index in range(8):
            if block_loc[self.color_name[color_index]] != []:
                cX_pixel = block_loc[self.color_name[color_index]][0]
                cY_pixel = block_loc[self.color_name[color_index]][1]
                level = block_loc[self.color_name[color_index]][4]
                world_coord_temp = np.dot(self.aff_matrix_diy,[cX_pixel,cY_pixel,1])
                cX = world_coord_temp[0]
                cY = world_coord_temp[1]
                if cY > y_limit:
                    if level != 0:
                        cZ = self.depth_level_to_hight(level)
                        angle = 0#self.coordinate_to_angle(cX,cY)
                        self.points_data[color_index] = [cX,cY,cZ,angle]
                    else:
                        print "Level is 0. Detection error. Depth is:",block_loc[self.color_name[color_index]][2]
            
    def coordinate_to_angle(self,x,y,method):
        if method ==1:
            angle_temp = np.arctan2(y,x)
            if x>=0 and y >=0:
                angle = np.pi/2 - angle_temp
            elif x<0 and y>=0:
                angle = -angle_temp + np.pi/2
            elif x<0 and y<0:
                angle = -3*np.pi/2 - angle_temp
            elif x>=0 and y<0:
                angle = np.pi/2 - angle_temp
            else:
                print "impossible happened."
        elif method == 2:
            angle = np.arctan2(x,y)
        return angle 
    def depth_level_to_hight(self,level):
        #print "level:",level
        if level == 1:
            cZ = 19
        elif level == 2:
            cZ = 57
        elif level == 3:
            cZ = 95
        return cZ + Z_DEPTH_COMPENSATION

    ## pick point from points data queue
    def run_point_list(self,method): 
        if method == 1:  # color based
            #print "Color order based."
            for color_index in range(8):
                if self.points_data[color_index] != []:
                    running_point = self.points_data[color_index]
                    self.points_data[color_index] = []
                    return running_point,color_index
            return 0,0 # means self.points_data finished
        if method == 2:  # hight based
            #print "hight order based."
            for color_index in range(8):
                if (self.points_data[color_index] != []) and (self.points_data[color_index][2] > 19 + Z_DEPTH_COMPENSATION):
                    running_point = self.points_data[color_index]
                    self.points_data[color_index] = []
                    return running_point,color_index
            return 0,0 # means self.points_data finished
    
    ### get one series of actions
    def task_one_step(self,pose,pose_final):
        count = 0
        self.job_finished = 0
        while True:
            if self.job_finished == -1:
                print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.3
                torque = 0.7
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint 1.     ++++++++++++++"
                speed = 0.3
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}   
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
                print "++++++++++++++    start arrive 2.     ++++++++++++++"
                speed = 0.3
                torque = 0.6
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd_pre}
                temp_pose = [pose_final[0],pose_final[1],pose_final[2],0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 4:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint 2.     ++++++++++++++"
                speed = 0.8
                torque = 0.8
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 6:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break

    def task_one_step_for_event_3(self,pose,final_pose):#
        count = 0
        self.job_finished = 0        
        while True:
            if self.job_finished == -1:
                print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.1
                torque = 0.6
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
                print "++++++++++++++    arrive transition point.     ++++++++++++++"
                speed = 0.1
                torque = 0.6
                temp_pose = [self.transition_point_3[0],self.transition_point_3[1],self.transition_point_3[2],self.transition_point_3[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count ==4:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
                print "++++++++++++++  above transition point .   ++++++++++++++"
                speed = 0.4
                torque = 0.8
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd_pre}
                temp_pose = [self.transition_point_3[0],self.transition_point_3[1],self.transition_point_3[2]+BLOCK_HIGH+10,0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 6:
                self.job_finished = 0
                print "++++++++++++++  transition point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                temp_pose = [self.transition_point_3[0],self.transition_point_3[1],self.transition_point_3[2],0, 1,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count ==7:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 8:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.4
                torque = 0.6
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 9:
                self.job_finished = 0
                print "++++++++++++++  above final point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd_pre}
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+BLOCK_HIGH,0,2,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 10:
                self.job_finished = 0
                print "++++++++++++++  arrive final point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                temp_pose = [final_pose[0],final_pose[1],final_pose[2],0,2,speed,torque]
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd}
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 11:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 12:
                self.job_finished = 0
                print "++++++++++++++  above final point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd_pre}
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+20,0,2,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 13:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.2
                torque = 0.6
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 14:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break

    def task_one_step_4_pre(self,pose,pose_final):
        count = 0 
        self.job_finished = 0       
        while True:
            if self.job_finished == -1:
                print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.2
                torque = 0.6
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1


            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint 1.     ++++++++++++++"
                speed = 0.3
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
                print "++++++++++++++    start arrive 2.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                temp_pose = [pose_final[0],pose_final[1],pose_final[2],pose_final[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count == 4:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint 2.     ++++++++++++++"
                speed = 0.4
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count == 6:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break

    def task_one_step_for_event_4(self,pose,final_pose):#
        count = 0  
        self.job_finished = 0      
        while True:
            if self.job_finished == -1:
                print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
                print "++++++++++++++    arrive   transition point.     ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                speed = 0.2
                torque = 0.6
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2],self.transition_point[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}                
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 4:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
                print "++++++++++++++  above transition point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd_pre}
                speed = 0.8
                torque = 0.8
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2]+BLOCK_HIGH,0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 6:
                self.job_finished = 0
                print "++++++++++++++  transition point .   ++++++++++++++"
                speed = 0.7
                torque = 0.8
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2],0, 1,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count ==7:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 8:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.5
                torque = 0.6
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 9:
                self.job_finished = 0
                print "++++++++++++++  above final point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+BLOCK_HIGH-10,0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
                time.sleep(1)
            if self.job_finished == 1 and count == 10:
                self.job_finished = 0
                speed = 0.1
                torque = 0.6
                print "++++++++++++++  arrive final point .   ++++++++++++++"
                temp_pose = [final_pose[0],final_pose[1],final_pose[2],0,0,speed,torque]
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd}
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 11:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 12:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.8
                torque = 0.8
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 13:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break

    def task_one_step_for_event_4_last_step_m2(self,pose,final_pose):
        count = 0  
        self.job_finished = 0      
        while True:
            if self.job_finished == -1:
                print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.3
                torque = 0.7
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1

            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
                print "++++++++++++++    arrive   transition point.     ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                speed = 0.2
                torque = 0.6
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2],self.transition_point[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}                
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 4:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
                print "++++++++++++++  above transition point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd_pre}
                speed = 0.8
                torque = 0.8
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2]+BLOCK_HIGH,0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 6:
                self.job_finished = 0
                print "++++++++++++++  transition point .   ++++++++++++++"
                speed = 0.7
                torque = 0.8
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                temp_pose = [self.transition_point[0],self.transition_point[1],self.transition_point[2],0, 1,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count ==7:
                self.job_finished = 0
                print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 8:
                self.job_finished = 0
                print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.5
                torque = 0.6
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 9:
                self.job_finished = 0
                print "++++++++++++++  above final point .   ++++++++++++++"
                speed = 0.1
                torque = 0.6
                temp_pose = [final_pose[0],final_pose[1]+1,final_pose[2]+25,np.pi,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":2}
                self.worker_job(worker_task)
                count += 1
                time.sleep(1)
            if self.job_finished == 1 and count == 10:
                self.job_finished = 0
                speed = 0.1
                torque = 0.6
                print "++++++++++++++  arrive final point .   ++++++++++++++"
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+15,np.pi,0,speed,torque]
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd}
                worker_task = {"task":"arrive","pose":temp_pose,"method":2}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 11:
                self.job_finished = 0
                print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 12:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break                

    def task_one_step_for_event_5(self,pose,final_pose):#
        count = 0   
        self.job_finished = 0     
        while True:
            if self.job_finished == -1:
                #print "action failed."
                self.task_one_step_unfinished = 0
                break
            if  count == 0:
                #print "++++++++++++++    start arrive 1.     ++++++++++++++"
                speed = 0.3
                torque = 0.8
                temp_pose = [pose[0],pose[1],pose[2],pose[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1                
            if self.job_finished == 1 and count ==1:
                self.job_finished = 0
                #print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 2:
                self.job_finished = 0
                #print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.3
                torque = 0.8
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 3:
                self.job_finished = 0
               # print "++++++++++++++    arrive transition point.     ++++++++++++++"
                speed = 0.2
                torque = 0.7
                temp_pose = [self.transition_point_5[0],self.transition_point_5[1],self.transition_point_5[2],self.transition_point_5[3],0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 4:
                self.job_finished = 0
                #print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 5:
                self.job_finished = 0
               # print "++++++++++++++  above transition point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd_pre}
                speed = 0.5
                torque = 0.8
                temp_pose = [self.transition_point_5[0],self.transition_point_5[1],self.transition_point_5[2]+BLOCK_HIGH,0,0,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 6:
                self.job_finished = 0
               # print "++++++++++++++   transition point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":self.transition_point_align_cmd}
                speed = 0.5
                torque = 0.8
                temp_pose = [self.transition_point_5[0],self.transition_point_5[1],self.transition_point_5[2],0, 1,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count ==7:
                self.job_finished = 0
               # print "++++++++++++++    start grab.     ++++++++++++++"
                worker_task = {"task":"grab"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 8:
                self.job_finished = 0
               # print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.4
                torque = 0.8
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 9:
                self.job_finished = 0
               # print "++++++++++++++  above final point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd_pre}
                speed = 0.09
                torque = 0.6
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+BLOCK_HIGH,0,2,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 10:
                self.job_finished = 0
               # print "++++++++++++++  arrive final point .   ++++++++++++++"
                speed = 0.09
                torque = 0.6
                temp_pose = [final_pose[0],final_pose[1],final_pose[2],0,2,speed,torque]
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd}
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 11:
                self.job_finished = 0
               # print "++++++++++++++    start drop.     ++++++++++++++"
                worker_task = {"task":"drop"}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 12:
                self.job_finished = 0
               # print "++++++++++++++  above final point .   ++++++++++++++"
                #worker_task = {"task":"cmd","joint angles":pose_final_cmd_pre}
                speed = 0.09
                torque = 0.8
                temp_pose = [final_pose[0],final_pose[1],final_pose[2]+BLOCK_HIGH+10,0,2,speed,torque]
                worker_task = {"task":"arrive","pose":temp_pose,"method":3}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 13:
                self.job_finished = 0
               # print "++++++++++++++    start arrive waypoint.     ++++++++++++++"
                speed = 0.8
                torque = 0.8
                worker_task = {"task":"waypoint","speed":speed,"torque":torque}
                self.worker_job(worker_task)
                count += 1
            if self.job_finished == 1 and count == 14:
                self.task_one_step_unfinished = 0
                print "----------------One step finished ----------------------"
                break

    def point_to_method(self,pose):
        dist = np.sqrt(pose[0]**2+pose[1]**2)
        print "Distance from block point is ",dist
        if (dist>METHOD_DISTANCE_LINE_INNER) and (dist<METHOD_DISTANCE_LINE_OUTER):
            print "Use method 2."
            return 2
        else:
            print "Use method 1."
            return 1         
     
    def event_1(self):
        event_1_t = threading.Thread(target=self.event_1_thread)
        event_1_t.start()

    def event_1_thread(self):
        self.video_mark = 0
        print "--------------------Event 1 start --------------------------"
        i = 0
        while True:
            try:
                self.get_point_from_img_method3(1)
                pose,color_index = self.run_point_list(1)#[100,-100,50,3*np.pi/4]
                if pose !=0:
                    i+=1
                    pose_final = [pose[0],-pose[1],pose[2],0]
                    task_one_step_thread = threading.Thread(target=self.task_one_step,args=(pose,pose_final))
                    task_one_step_thread.start()
                    while self.task_one_step_unfinished:
                        pass
                    self.task_one_step_unfinished = 1
                    print "Finished %d blocks." %i
            except:
                pass
            if i ==3:
                break
        print "---------------end of event 1-----------------"
        self.video_mark = 1

    def event_2(self):
        event_2_thread = threading.Thread(target=self.event_2_thread)
        event_2_thread.start()

    def event_2_thread(self):
        self.video_mark = 0
        print "--------------------Event 2 start --------------------------"
        i = 0
        while True:
            try:
                self.get_point_from_img_method3(1)
                print "points_data:",self.points_data
                pose,color_index = self.run_point_list(1)#[100,-100,50,3*np.pi/4]
                print "pose:",pose, color_index
                if pose !=0:
                    i+=1
                    pose_final = [0,-200,pose[2]+(i-1)*BLOCK_HIGH+7,0]
                    task_one_step_thread = threading.Thread(target=self.task_one_step,args=(pose,pose_final))
                    task_one_step_thread.start()
                    while self.task_one_step_unfinished:
                        pass
                    self.task_one_step_unfinished = 1
                    print "Finished %d blocks." %i
            except:
                pass
            if i ==3:
                break
        print "---------------end of event 2-----------------"
        self.video_mark = 1

    def event_3(self):
        delta = 10+BLOCK_HIGH
        left_most_x = -220
        down_most_y = -165
        self.task_3_final_point = [[left_most_x,down_most_y,19,0],\
        [left_most_x+1*delta-0.5,down_most_y,19,0],\
        [left_most_x+2*delta,down_most_y,19,0],\
        [left_most_x+3*delta,down_most_y,19,0],\
        [left_most_x+4*delta,down_most_y,19,0],\
        [left_most_x+5*delta,down_most_y,19,0],\
        [left_most_x+6*delta,down_most_y,19,0],\
        [left_most_x+7*delta,down_most_y,19,0]]
        event_3_thread = threading.Thread(target=self.event_3_thread)
        event_3_thread.start()

    def event_3_thread(self):
        self.video_mark = 0
        print "--------------------Event 3 start --------------------------"
        i = 0
        self.task_one_step_unfinished = 1
        while True:
            try:
                self.get_point_from_img_method3(2)# y>-140 designed for Event 3
                pose,color_index = self.run_point_list(1)# color
                #print "qwdqwe",pose, color_index
                if pose !=0:
                    i+=1
                    pose_final = self.task_3_final_point[color_index]
                    task_one_step_thread_3 = threading.Thread(target=self.task_one_step_for_event_3,args=(pose,pose_final))
                    task_one_step_thread_3.start()
                    while self.task_one_step_unfinished:
                        pass
                    self.task_one_step_unfinished = 1
                    print "Finished %s block." %self.color_name[color_index]
            except:
                pass
          #  if i == 8:
              #  print "Get 8 blocks."
               # break
        print "--------------------Event 3 end --------------------------"
        self.video_mark = 1
        
    def event_4(self):
        x = 0
        y = -160
        z = 20
        self.task_4_final_point = [[x,y,z,0],\
        [x,y,z+BLOCK_HIGH,0],\
        [x,y,z+2*BLOCK_HIGH,0],\
        [x,y,z+3*BLOCK_HIGH,0],\
        [x,y,z+4*BLOCK_HIGH,0],\
        [x,y,z+5*BLOCK_HIGH,0],\
        [x,y+8,z+6*BLOCK_HIGH,0],\
        [x,y+17,z+7*BLOCK_HIGH+1,0]]

        y_line = -115
        self.task_4_pre_set = [[-240,y_line,z,0],\
        [-100,250,z,0],\
        [0,280,z,0],\
        [240,y_line,z,0],\
        [-150,y_line+20,z,0],\
        [-175,200,z,0]]
        
        x = -150
        y_line = -220
        delta = 70
        self.task_4_pre_set_2 = [[0,100,19,0],\
        [100,0,19,0],\
        [-100,0,19,0],\
        [-175,-130,0,0],\
        [175,-130,0,0]]
        
    
        
        
        event_4_thread = threading.Thread(target=self.event_4_thread)
        event_4_thread.start()

    def event_4_thread(self):
        self.video_mark = 0

        y_limit = -154
        print "--------------------Event 4 start --------------------------"
        ## fist find the number of stack blocks
        while True:
            level = 0
            stack_num = 0
            frame = freenect.sync_get_video()[0]
            frame_d = freenect.sync_get_depth()[0]
            block_loc = self.img_proc_dyn.get_all_coord(frame,frame_d)
            #print "block_loc:",block_loc
            for color_index in range(8):
                if block_loc[self.color_name[color_index]] != []:
                    cX_pixel = block_loc[self.color_name[color_index]][0]
                    cY_pixel = block_loc[self.color_name[color_index]][1]
                    level_temp = block_loc[self.color_name[color_index]][4]
                    world_coord_temp = np.dot(self.aff_matrix_diy,[cX_pixel,cY_pixel,1])
                    cX = world_coord_temp[0]
                    cY = world_coord_temp[1]
                    if cY > y_limit:
                        if level_temp != 0:
                            level += level_temp
                            if level_temp > 1:
                                stack_num += (level_temp-1)
                        else:
                            print "Level is 0. Detection error. Depth is:",block_loc[self.color_name[color_index]][2]
            if level==8:
                print "Stack number is:",stack_num
                break
        ## put all blocks in one level hignt
        j = 0
        while True:
            print "Looking to relocate."
            try:
                self.get_point_from_img_method3(3)# y>-150 ,designed for Event 4
                pose,color_index = self.run_point_list(2)# find hight > 19 + Z_depth_compensation
                if pose !=0:
                    pose_final = self.task_4_pre_set[j]#self.task_4_pre_set[j]
                    j += 1
                    task_one_step_thread_4 = threading.Thread(target=self.task_one_step_4_pre,args=(pose,pose_final))
                    task_one_step_thread_4.start()
                    while self.task_one_step_unfinished:
                        pass
                    self.task_one_step_unfinished = 1
                    print "Re-locate %s block." %self.color_name[color_index]
                #else:
                   # break
                if j == stack_num:  # this has to be known
                    break
            except:
                pass
        
        print "--------------All blocks are one level now.---------------------"
        ## stack
        i = 0
        while True:
            try:
                self.get_point_from_img_method3(3)#>-180
                pose,color_index = self.run_point_list(1)#color based order
                if color_index == i and pose !=0:
                    if color_index < 7:
                        i+=1
                        pose_final = self.task_4_final_point[color_index]
                        task_one_step_thread_4 = threading.Thread(target=self.task_one_step_for_event_4,args=(pose,pose_final))
                        task_one_step_thread_4.start()
                        while self.task_one_step_unfinished:
                            pass
                        self.task_one_step_unfinished = 1
                 
                    elif color_index == 7:
                        i+=1
                        pose_final = self.task_4_final_point[color_index]
                        task_one_step_thread_4 = threading.Thread(target=self.task_one_step_for_event_4_last_step_m2,args=(pose,pose_final))
                        task_one_step_thread_4.start()
                        while self.task_one_step_unfinished:
                            pass
                        self.task_one_step_unfinished = 1
                  
                    print "Finished %s block." %self.color_name[color_index]
                elif color_index != i:
                    pass
                    #print "Waiting for %s block."%self.color_name[i]
            except:
                pass
            if i == 8:
                print "Get 8 blocks."
                break
        print "--------------------Event 4 end --------------------------"
        self.video_mark = 1

    def event_5(self):
        x=-220
        y=-190#-170
        z=20
        delta = 8#12
        l = (delta + BLOCK_HIGH)
        self.task_5_final_point_2 = [[x,y,z,0],\

        [x+l,y,z,0],\
        [x+l/2,y,z+BLOCK_HIGH,0],\

        [x+2*l,y,z,0],\
        [x+l/2+l,y,z+BLOCK_HIGH,0],\
        [x+l,y,z+2*BLOCK_HIGH,0],\

        [x+3*l,y,z,0],\
        [x+l/2+2*l,y,z+BLOCK_HIGH,0],\
        [x+2*l,y,z+2*BLOCK_HIGH,0],\
        [x+3*l/2,y,z+3*BLOCK_HIGH,0],\

        [x+4*l,y,z,0],\
        [x+l/2+3*l,y,z+BLOCK_HIGH,0],\
        [x+3*l,y,z+2*BLOCK_HIGH,0],\
        [x+5*l/2,y,z+3*BLOCK_HIGH,0],\
        [x+2*l,y,z+4*BLOCK_HIGH,0],\

        [x+5*l,y,z,0],\
        [x+l/2+4*l,y,z+BLOCK_HIGH,0],\
        [x+4*l,y,z+2*BLOCK_HIGH,0],\
        [x+7*l/2,y,z+3*BLOCK_HIGH,0],\
        [x+3*l,y,z+4*BLOCK_HIGH,0],\
        [x+5*l/2,y,z+5*BLOCK_HIGH,0],\

        [x+6*l,y,z,0],\
        [x+l/2+5*l,y,z+BLOCK_HIGH,0],\
        [x+5*l,y,z+2*BLOCK_HIGH,0],\
        [x+9*l/2,y,z+3*BLOCK_HIGH,0],\
        [x+4*l,y,z+4*BLOCK_HIGH,0],\
        [x+7*l/2,y,z+5*BLOCK_HIGH,0],\
        [x+3*l,y,z+6*BLOCK_HIGH,0],\

        [x+7*l,y,z,0],\
        [x+l/2+6*l,y,z+BLOCK_HIGH,0],\
        [x+6*l,y,z+2*BLOCK_HIGH,0],\
        [x+11*l/2,y,z+3*BLOCK_HIGH,0],\
        [x+5*l,y,z+4*BLOCK_HIGH,0],\
        [x+9*l/2,y,z+5*BLOCK_HIGH,0],\
        [x+4*l,y,z+6*BLOCK_HIGH,0],\
        [x+7*l/2,y,z+6*BLOCK_HIGH,0]]

        event_5_thread = threading.Thread(target=self.event_5_thread)
        event_5_thread.start()

    def event_5_thread(self):
        self.video_mark = 0
        print "--------------------Event 5 start --------------------------"
        i = 0
        self.task_one_step_unfinished = 1
        while True:
            try:
                self.get_point_from_img_method3(1)# y>0?????
                pose,color_index = self.run_point_list(1)# color
                if pose !=0:
                    pose_final = self.task_5_final_point_2[i]
                    i+=1
                    print "i:",i
                    task_one_step_thread_5 = threading.Thread(target=self.task_one_step_for_event_5,args=(pose,pose_final))
                    task_one_step_thread_5.start()
                    while self.task_one_step_unfinished:
                        pass
                    self.task_one_step_unfinished = 1
                    print "Locate %d blocks."%i
            except:
                pass

        print "--------------------Event 5 end --------------------------"
        self.video_mark = 1
        
    def play(self):
        """ 
        Play Funtion
        Continuously called by GUI 
        """
        """ Renders the Video Frame """
        if self.video_mark == 1:
            try:
                self.video.captureVideoFrame()
                self.video.captureDepthFrame()
                #self.video.blobDetector()

                if(self.ui.radioVideo.isChecked()):
                    self.ui.videoFrame.setPixmap(self.video.convertFrame())

                if(self.ui.radioDepth.isChecked()):
                    self.ui.videoFrame.setPixmap(self.video.convertDepthFrame())

                if (self.ui.radioInfo.isChecked()):
                    self.switch_video()

                if (self.ui.radioDyn.isChecked()):
                    self.switch_video_dyn()
                #self.ui.videoFrame.setScaledContents(True)
            except TypeError:
                print "No frame"
        """ 
        Update GUI Joint Coordinates Labels
        LAB TASK: include the other slider labels 
        """
        self.ui.rdoutBaseJC.setText(str("%.2f" % (self.rex.joint_angles_fb[0])))
        self.ui.rdoutShoulderJC.setText(str("%.2f" % (self.rex.joint_angles_fb[1])))
        self.ui.rdoutElbowJC.setText(str("%.2f" % (self.rex.joint_angles_fb[2])))
        self.ui.rdoutWristJC.setText(str("%.2f" % (self.rex.joint_angles_fb[3])))
        ## Update forward kinematics info in GUI
        self.end_effector = self.rex.rexarm_FK(self.rex.joint_angles_fb)#joint_angles_fb not joint_angles, for fb is more accurate
        self.ui.rdoutX.setText(str("%.2f"% self.end_effector[0]))
        self.ui.rdoutY.setText(str("%.2f"%self.end_effector[1]))
        self.ui.rdoutZ.setText(str("%.2f"%self.end_effector[2]))
        self.ui.rdoutT.setText(str("%.2f"%self.end_effector[3]))#unit converter
        """ 
        Mouse position presentation in GUI
        TO DO: after getting affine calibration make the apprriate label
        to present the value of mouse position in world coordinates 
        """    
        x = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).x()
        y = QtGui.QWidget.mapFromGlobal(self,QtGui.QCursor.pos()).y()
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            ## multi processes flag mark
            z = self.video.currentDepthFrame[y][x]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (x,y,z))

            if (self.video.aff_flag == 2):
                """ TO DO Here is where affine calibration must be used """
                coord_temp = np.ones((3,1))
                coord_temp[0] = x
                coord_temp[1] = y
                world_temp_aff = np.dot(self.aff_matrix_diy,coord_temp)
                ## multi processes flag mark
                self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (world_temp_aff[0],world_temp_aff[1],z*DEPTH_RATIO))
            else:
                self.ui.rdoutMouseWorld.setText("(-,-,-)")

        """ 
        Updates status label when rexarm playback is been executed.
        This will be extended to includ other appropriate messages
        """ 
        if(self.rex.plan_status == 1):
            self.ui.rdoutStatus.setText("Playing Back - Waypoint %d"
                                    %(self.rex.wpt_number + 1))
        # Check if arm end effecter reach certain point (for teach and repeat)
        if self.start_teach_repeat_in_play == 1:
            self.teach_repeat_publish_check()


    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position 
        TO DO: Implement for the other sliders 
        """
        #print "slider changed.\n"
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value()))

        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")

        self.rex.max_torque = self.ui.sldrMaxTorque.value()/100.0
        for i in range(self.rex.num_joints):
            self.rex.speed_list[i] = self.ui.sldrSpeed.value()/100.0
        self.rex.joint_angles[0] = self.ui.sldrBase.value()*D2R
        self.rex.joint_angles[1] = self.ui.sldrShoulder.value()*D2R
        self.rex.joint_angles[2] = self.ui.sldrElbow.value()*D2R
        self.rex.joint_angles[3] = self.ui.sldrWrist.value()*D2R
        self.rex.joint_angles[4] = self.ui.sldrGrip1.value()*D2R
        self.rex.joint_angles[6] = self.ui.sldrGrip2.value()*D2R
        
        self.rex.cmd_publish()


    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for 
        affine calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.last_click[0] = x - MIN_X
        self.last_click[1] = y - MIN_Y
        
        """ If affine calibration is performed """
        if (self.video.aff_flag == 1):
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),
                                                                 (y-MIN_Y)]

            """ Update the number of used poitns for calibration """
            self.video.mouse_click_id += 1

            """ Update status label text """
            self.ui.rdoutStatus.setText("Affine Calibration: Click Point %d" 
                                      %(self.video.mouse_click_id + 1))

            """ 
            If the number of click is equal to the expected number of points
            computes the affine calibration.
            
            LAB TASK: Change this code to use your affine calibration routine
            and NOT openCV pre-programmed function as it is done now.
            """
            if(self.video.mouse_click_id == self.video.aff_npoints):

                """ Perform affine calibration with OpenCV """
                aff_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord[:3],
                                        self.video.real_coord[:3])
                # perform by diy multi points affine calibration
                self.aff_matrix_diy = self.video.affine_transform_diy(
                                        self.video.mouse_coord,
                                        self.video.real_coord)

                # calculate the difference between my own affine calibration in video.py and openCV pre-programmed function
                diff_aff_matrix = aff_matrix - self.aff_matrix_diy

                """ Updates Status Label to inform calibration is done """ 
                self.ui.rdoutStatus.setText("Begin calibrate depth camera.")

                """ 
                print affine calibration matrix numbers to terminal
                """ 
                print "openCV aff_matrix: \n", aff_matrix, \
                "\n self-designed aff_matrix: \n", self.aff_matrix_diy, \
                "\n Diff of the two matrices: \n", diff_aff_matrix,"\n"
                """ 
                Update status of calibration flag and number of mouse
                clicks
                """
                self.video.aff_flag = 2
                self.video.mouse_click_id = 0
                self.affine_cal_depth()
                return

        ## for depth, click on GRB 
        if (self.video.aff_dep_flag == 1):
            """ Save last mouse coordinate """
            self.video.mouse_coord[self.video.mouse_click_id] = [(x-MIN_X),
                                                                 (y-MIN_Y)]
            self.video.mouse_click_id += 1
            print "mouse",self.video.mouse_coord
            print self.video.mouse_click_id
            if self.video.mouse_click_id < self.video.aff_npoints:
                self.ui.rdoutStatus.setText("Affine Calibration (on RGB image): Click Point %d" 
                                      %(self.video.mouse_click_id + 1))
            else:
                self.ui.rdoutStatus.setText("Affine Calibration (on depth image): Click Point 1")
            if(self.video.mouse_click_id == self.video.aff_npoints):
                self.video.aff_dep_flag = 2
                self.video.mouse_click_id = 0
                return
        ## for depth, click on depth
        if (self.video.aff_dep_flag == 2):
            """ Save last mouse coordinate """
            self.video.depth_coord[self.video.mouse_click_id] = [(x-MIN_X),
                                                                 (y-MIN_Y)]
            self.video.mouse_click_id += 1
            print "depth:",self.video.depth_coord
            print self.video.mouse_click_id
            self.ui.rdoutStatus.setText("Affine Calibration (on depth image): Click Point %d" 
                                      %(self.video.mouse_click_id + 1))
            if(self.video.mouse_click_id == self.video.aff_npoints):
                print "mouse depth final:","\n",self.video.mouse_coord,"\n",self.video.depth_coord
                """ Perform affine calibration with OpenCV """
                aff_d_matrix = cv2.getAffineTransform(
                                        self.video.mouse_coord[:3],
                                        self.video.depth_coord[:3])
                # perform by diy multi points affine calibration
                self.aff_d_matrix_diy = self.video.affine_transform_rgb_d_diy(
                                        self.video.mouse_coord,
                                        self.video.depth_coord)

                # calculate the difference between my own affine calibration in video.py and openCV pre-programmed function
                diff_aff_d_matrix = aff_d_matrix - self.aff_d_matrix_diy
                self.ui.rdoutStatus.setText("Finished depth camera calibration.")
                print "openCV aff_matrix for depth: \n", aff_d_matrix, \
                "\n self-designed aff_matrix for depth: \n", self.aff_d_matrix_diy, \
                "\n Diff of the two matrices for depth: \n", diff_aff_d_matrix,"\n"
                self.video.aff_dep_flag = 3
                self.video.mouse_click_id = 0
                return

    def affine_cal(self):
        """ 
        Function called when affine calibration button is called.
        Note it only change the flag to record the next mouse clicks
        and updates the status text label 
        """
        print "Clibration step 1: Affine from RGB to real world. pick points in grey board. from leftabove, counterclockwise."
        self.video.aff_flag = 1 
        self.ui.rdoutStatus.setText("Affine Calibration from RGB to real world: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))
    def affine_cal_depth(self):
        print "Calibration step 2: affine from RGB to Depth"
        self.video.aff_dep_flag = 1
        self.video.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0], [0.0, 0.0]])
        self.ui.rdoutStatus.setText("Affine Calibration from RGB to Depth: Click Point %d" 
                                    %(self.video.mouse_click_id + 1))
        print self.video.mouse_coord
        print self.video.mouse_click_id 

    def teach_repeat_clear(self):
        self.rex.num_joints = 4
        self.record_B = []
        self.record_S = []
        self.record_E = []
        self.record_W = []

        print "Count and data list cleared.\n"

    def teach_repeat_record(self):
 
        self.record_B.append(self.rex.joint_angles_fb[0]) #joint_angles_fb unit: radius
        self.record_S.append(self.rex.joint_angles_fb[1])
        self.record_E.append(self.rex.joint_angles_fb[2])
        self.record_W.append(self.rex.joint_angles_fb[3])
        print "Count:  recorded", (self.teach_repeat_count)
        print "Record position: ", float(self.record_B[-1])*R2D,float(self.record_S[-1])*R2D,float(self.record_E[-1])*R2D,float(self.record_W[-1])*R2D

    def teach_repeat_publish_initial(self):
        teach_repeat_thread  = threading.Thread(target=self.teach_repeat_publish)
        teach_repeat_thread.start()

    def teach_repeat_publish(self):
        print "##############  teach and repeat normal   ##############\n"
        self.time_repeat = []
        self.rex.max_torque = 0.4
        tol_check = 0.05
        for j in range(4):
            self.rex.speed_list[j] = 0.09 #self.ui.sldrSpeed.value()/100
        #time_0 = time.time()
        for i in range(len(self.record_B)):
            print "count:",i 
            self.rex.joint_angles[0] = self.record_B[i]#self.ui.sldrBase.value()*D2R
            self.rex.joint_angles[1] = self.record_S[i]#self.ui.sldrShoulder.value()*D2R
            self.rex.joint_angles[2] = self.record_E[i]#self.ui.sldrElbow.value()*D2R
            self.rex.joint_angles[3] = self.record_W[i]#self.ui.sldrWrist.value()*D2R
            self.rex.num_joints = 4
            self.rex.cmd_publish()
            while True:
               # print "%.2f,%.2f,%.2f,%.2f"%(abs(float(self.record_B[i])-self.rex.joint_angles_fb[0]),\
               # abs(float(self.record_S[i])-self.rex.joint_angles_fb[1]),\
               # abs(float(self.record_E[i])-self.rex.joint_angles_fb[2]),\
               # abs(float(self.record_W[i])-self.rex.joint_angles_fb[3]))
                time.sleep(0.00001)

                if abs(self.record_B[i]-self.rex.joint_angles_fb[0])<tol_check and\
                abs(self.record_S[i]-self.rex.joint_angles_fb[1])<tol_check and\
                abs(self.record_E[i]-self.rex.joint_angles_fb[2])<tol_check and\
                abs(self.record_W[i]-self.rex.joint_angles_fb[3])<tol_check:
                    self.time_repeat.append(time.time()*1000)
                    break
        print "Teach and repeat finished."

        #np.savetxt('./data/waypoint_B.csv',self.record_B,delimiter=',')
       #np.savetxt('./data/waypoint_S.csv',self.record_S,delimiter=',')
       # np.savetxt('./data/waypoint_E.csv',self.record_E,delimiter=',')
       # np.savetxt('./data/waypoint_W.csv',self.record_W,delimiter=',')
       # np.savetxt('./data/timestamp.csv',self.time_repeat,delimiter=',')
		
    def teach_repeat_publish_cubic_initial(self):
        teach_repeat_cubic_thread  = threading.Thread(target=self.teach_repeat_publish_cubic)
        teach_repeat_cubic_thread.start()   

    def teach_repeat_publish_cubic(self):
        print "#############   teach and repeat cubic spline   ###############\n"
        self.record_B_cubic = []
        self.record_S_cubic = []
        self.record_E_cubic = []
        self.record_W_cubic = []

        self.record_B_cubic_speed = []
        self.record_S_cubic_speed = []
        self.record_E_cubic_speed = []
        self.record_W_cubic_speed = []

        self.time_cubic_spline = []
        self.teach_repeat_cubic_spline()
        tol_check = 0.05
        for j in range(4):
            self.rex.max_torque = 0.4
            self.rex.speed_list[j] = 0.04
        #time_0 = time.time()
        for i in range(len(self.record_B_cubic)):
            print "count:",i
            self.rex.joint_angles[0] = self.record_B_cubic[i]#self.ui.sldrBase.value()*D2R
            self.rex.joint_angles[1] = self.record_S_cubic[i]#self.ui.sldrShoulder.value()*D2R
            self.rex.joint_angles[2] = self.record_E_cubic[i]#self.ui.sldrElbow.value()*D2R
            self.rex.joint_angles[3] = self.record_W_cubic[i]#self.ui.sldrWrist.value()*D2R
            """
            self.rex.speed_list[0] = float(self.record_B_cubic_speed[i])#self.ui.sldrBase.value()*D2R
            self.rex.speed_list[1] = float(self.record_S_cubic_speed[i])#self.ui.sldrShoulder.value()*D2R
            self.rex.speed_list[2] = float(self.record_E_cubic_speed[i])#self.ui.sldrElbow.value()*D2R
            self.rex.speed_list[3] = float(self.record_W_cubic_speed[i])#self.ui.sldrWrist.value()*D2R
            """
            self.rex.num_joints = 4
            self.rex.cmd_publish()
            while True:
                
            #    print "%.2f,%.2f,%.2f,%.2f"%(abs(float(self.record_B_cubic[i])-self.rex.joint_angles_fb[0]),\
            #    abs(float(self.record_S_cubic[i])-self.rex.joint_angles_fb[1]),\
            #    abs(float(self.record_E_cubic[i])-self.rex.joint_angles_fb[2]),\
              #  abs(float(self.record_W_cubic[i])-self.rex.joint_angles_fb[3]))
                
                time.sleep(0.00001)
            	if abs(self.record_B_cubic[i]-self.rex.joint_angles_fb[0])<tol_check and\
            	abs(self.record_S_cubic[i]-self.rex.joint_angles_fb[1])<tol_check and\
            	abs(self.record_E_cubic[i]-self.rex.joint_angles_fb[2])<tol_check and\
            	abs(self.record_W_cubic[i]-self.rex.joint_angles_fb[3])<tol_check:
                    self.time_cubic_spline.append(time.time()*1000)
                    break
        print "Teach and repeat finished."

     #   np.savetxt('./data/waypoint_cubic_B.csv',self.record_B_cubic,delimiter=',')
      #  np.savetxt('./data/waypoint_cubic_S.csv',self.record_S_cubic,delimiter=',')
      #  np.savetxt('./data/waypoint_cubic_E.csv',self.record_E_cubic,delimiter=',')
     #   np.savetxt('./data/waypoint_cubic_W.csv',self.record_W_cubic,delimiter=',')
     #   np.savetxt('./data/timestamp_cubic.csv',self.time_cubic_spline,delimiter=',')

    def teach_repeat_cubic_spline(self):
        v0=0
        t0 = 0
        vf=0
        tf = 1
        delta_t = 0.01#0.027  
        for i in range(len(self.record_B)-1):
            q0 = self.record_B[i]
            qf = self.record_B[i+1]
            self.cubic_spline_one(q0,v0,t0,qf,vf,tf,delta_t,self.record_B_cubic,self.record_B_cubic_speed)
        for i in range(len(self.record_S)-1):
            q0 = self.record_S[i]
            qf = self.record_S[i+1]
            self.cubic_spline_one(q0,v0,t0,qf,vf,tf,delta_t,self.record_S_cubic,self.record_S_cubic_speed)
        for i in range(len(self.record_E)-1):
            q0 = self.record_E[i]
            qf = self.record_E[i+1]
            self.cubic_spline_one(q0,v0,t0,qf,vf,tf,delta_t,self.record_E_cubic,self.record_E_cubic_speed)
        for i in range(len(self.record_W)-1):
            q0 = self.record_W[i]
            qf = self.record_W[i+1]
            self.cubic_spline_one(q0,v0,t0,qf,vf,tf,delta_t,self.record_W_cubic,self.record_W_cubic_speed)

    
    def angle_to_xyztheta(self):
        self.end_effector = []
        self.end_effector_cubic = []
        for i in range(len(self.record_B)):
            angle_temp = [self.record_B[i],self.record_S[i],self.record_E[i],self.record_W[i]]
            temp_array =  self.rex.rexarm_FK(angle_temp)
            temp_list = [temp_array[0][0],temp_array[1][0],temp_array[2][0],temp_array[3][0]]
            self.end_effector.append(temp_list)
        for i in range(len(self.record_B_cubic)):
            angle_temp = [self.record_B_cubic[i],self.record_S_cubic[i],self.record_E_cubic[i],self.record_W_cubic[i]]
            temp_array =  self.rex.rexarm_FK(angle_temp)
            temp_list = [temp_array[0][0],temp_array[1][0],temp_array[2][0],temp_array[3][0]]
            self.end_effector_cubic.append(temp_list)



    def draw(self):
        self.angle_to_xyztheta()
        """
        self.record_B
        self.record_S
        self.record_E
        self.record_W
        self.end_effector

        self.record_B_cubic
        self.record_S_cubic
        self.record_E_cubic
        self.record_W_cubic
        self.end_effector_cubic
        """

        x_normal = [x[0] for x in self.end_effector]
        y_normal = [x[1] for x in self.end_effector]
        z_normal = [x[2] for x in self.end_effector]
        time_normal =[x-self.time_repeat[0] for x in self.time_repeat]

        x_cubic = [x[0] for x in self.end_effector_cubic]
        y_cubic = [x[1] for x in self.end_effector_cubic]
        z_cubic = [x[2] for x in self.end_effector_cubic]
        time_cubic =[x-self.time_cubic_spline[0] for x in self.time_cubic_spline]



        plt.figure(1)
        plt.subplot(211)
        plt.plot(time_normal,self.record_B,'r-*',time_normal,self.record_S,'b-*',time_normal,self.record_E,'g-*',time_normal,self.record_W,'k-*')
        #plt.xlabel('time(ms)')
        plt.ylabel('angle(rad)')
        plt.legend(["B","S","E","W"])
        plt.title('Arm angle change')
        plt.subplot(212)
        plt.plot(time_cubic,self.record_B_cubic,'r-o',time_cubic,self.record_S_cubic,'b-o',time_cubic,self.record_E_cubic,'g-o',time_cubic,self.record_W_cubic,'k-o')
        plt.xlabel('time(ms)')
        plt.ylabel('angle(rad)')
        plt.legend(["B","S","E","W"])
        plt.title('Arm angle change(Cubic spline)')
        plt.savefig("BSEW.png")




        fig = plt.figure(2)
        ax = fig.gca(projection='3d')
        ax.plot(x_normal,y_normal,z_normal,'r-*',label='original traj.')
        ax.plot(x_cubic,y_cubic,z_cubic,'b-o',label='cubic spline traj.')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        #ax = plt.axes(projection='3d')
        #ax.scatter(x_normal,y_normal,z_normal,'-b')
        #plt.plot(x_normal,y_normal,z_normal,'r-o')
      #  plt.plot(time_normal,x_normal,'r-*',time_normal,y_normal,'b-*',time_normal,z_normal,'g-*')
       
        plt.legend()
        plt.title('movement traj.')
        plt.savefig("coordinates_3D.png")
        plt.show()


    def cubic_spline_one(self, q0,v0,t0,qf,vf,tf,delta_t,angle,speed):
        coefs = []
        split_num = int((tf-t0)/delta_t)
        a0 = q0
        a1 = v0
        a2 = (-vf*tf-2*v0*tf + 3*qf - 3*q0)/(tf)**2
        a3 = (vf*tf+v0*tf - 2*qf + 2*q0)/(tf)**3

      #  a0 = float(q0)
     #   a1 = float(v0)
       # a2 = (-float(vf)*float(tf)-2*float(v0)*float(tf) + 3*float(qf) - 3*float(q0))/(tf)**2
       # a3 = (float(vf)*float(tf)+float(v0)*float(tf) - 2*float(qf) + 2*float(q0))/(tf)**3

        cubic_temp = []
        #cubic_temp.append(float(q0))
        #cubic_temp.append(float(v0))
        #cubic_temp.append((3*(float(qf) -float(q0))-(2*float(v0)+float(vf))*(float(tf)-float(t0)))/(float(tf)-float(t0))/(float(tf)-float(t0)))
       # cubic_temp.append((2*(float(q0)-float(qf))+(float(v0)+float(vf))*(float(tf)-float(t0)))/(float(tf)-float(t0))/(float(tf)-float(t0)/(float(tf)-float(t0))))
       
       # print "a0-3:",a0,a1,a2,a3
        for i in range(split_num):
            t_i = i*delta_t
            angle.append(a0+a1*t_i+a2*t_i**2+a3*t_i**3)
            speed.append(a1+2*a2*t_i+3*a3*t_i**2)


def main():
    #QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    ex.show()
    # kill all processes
    app.aboutToQuit.connect(ex.shutdown_p)
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()
