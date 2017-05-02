import cv2
import numpy as np
import freenect
import imutils
import time
import threading
import os,shutil

class ImgProcess():
    def __init__(self):
        board_corner = []
        # block area and arcLength range
        self.block_area_min = 300
        self.block_area_max = 520
        self.block_len_min = 70
        self.block_len_max = 120
        self.img_scan = np.array([])
        #Closing: dilation followed by Erosion
        self.closing_kenel_size = 5
        self.closing_kernel = np.ones((self.closing_kenel_size,self.closing_kenel_size),np.uint8)
        
        #self.opening_kenel_size = 5
        #self.opening_kernel = np.ones((self.opening_kenel_size,self.opening_kenel_size),np.uint8)
        ## read file to get four board corner points in the manipulater board
        try:
            board_corner = np.loadtxt(open("./calibration/board_corner.csv"),delimiter = ",")
        except IOError:
            print "File can't be found, calibrate first."
        # build 480x640 image (grey scale, same resolution of Kinect camera)
        self.board_filter = np.zeros((480, 640),np.uint8)
        points = np.array(board_corner,np.uint32)#[[120,100],[490,100],[490,400],[120,400]])
        cv2.fillConvexPoly(self.board_filter,np.int32(points),(255,255,255))
        circle_center = tuple([int(x) for x in np.sum(points, axis=0)/4])
        cv2.circle(self.board_filter,circle_center,20,(0,255,255),-1)
        #cv2.imshow('board_filter',self.board_filter)
        self.mat_intr = np.array([[1/594.2,0,-339.5/594.2],[0,-1/591.0,242.7/591.0],[0,0,-1]])
        try:
            self.RGB2World = np.loadtxt(open("./calibration/calibration_matrix.csv"),delimiter = ",")
            self.RGB2depth = np.loadtxt(open("./calibration/calibration_matrix_depth.csv"),delimiter = ",")
        except IOError:
            print "File can't be found, calibrate first."

        depth_board_base = 720
        delta = 5
        self.depth_1_min = depth_board_base - 15 - delta
        self.depth_1_max = depth_board_base - 15 + delta
        self.depth_2_min = depth_board_base - 32 - delta
        self.depth_2_max = depth_board_base - 32 + delta
        self.depth_3_min = depth_board_base - 51 - delta
        self.depth_3_max = depth_board_base - 51 + delta

        lower_black = np.array([0,20,50])
        upper_black = np.array([180,100,85])

        lower_red = np.array([165,110,120])
        upper_red = np.array([185,245,180])#

        lower_orange = np.array([0,120,190])
        upper_orange = np.array([13,250,254])

        lower_yellow = np.array([0,0,250])
        upper_yellow = np.array([40,255,255])

        lower_green = np.array([40,40,105])
        upper_green = np.array([60,150,200])

        lower_blue = np.array([110,80,150])
        upper_blue = np.array([120,165,190])

        lower_violet = np.array([140,70,100])
        upper_violet = np.array([160,150,165])#

        lower_pink = np.array([160,140,205]) 
        upper_pink = np.array([185,220,254])

        self.color_db = [[lower_black, upper_black],
                         [lower_red, upper_red],
                         [lower_orange, upper_orange],
                         [lower_yellow, upper_yellow],
                         [lower_green, upper_green],
                         [lower_blue, upper_blue],
                         [lower_violet, upper_violet],
                         [lower_pink, upper_pink]]
        self.color_name = ["black","red","orange","yellow","green","blue","violet","pink"]
        self.color_contour_RGB = [[50,50,50],[150,0,0],[255,106,0],[255,255,45],[32,109,0],[0,57,150],[142,24,163]]
        #data of block location, dictionary value:[cX, cY, Depth]
        self.dict_init()
        #self.test_Main_func()
 
    def dict_init(self):
        self.block_loc = {"black":[],
        "red":[],
        "orange":[],
        "yellow":[],
        "green":[],
        "blue":[],
        "violet":[],
        "pink":[]
        }

    def test_Main_func(self):
        frame = freenect.sync_get_video()[0]
        frame_d = freenect.sync_get_video()[0]
        self.get_all_coord(frame,frame_d)
        #cv2.imshow("original",frame)
        while True:
            ch0 = cv2.waitKey(10)
            ch = 0xFF & ch0
            #if ch == 32:
            frame = freenect.sync_get_video()[0]
            frame_d = freenect.sync_get_depth()[0]
            self.get_one_coord(frame,frame_d,1)
            #frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
            #self.draw(frame, self.block_loc)
            if ch == 27:
                print "ESC detected.\n"
                break 

    def get_all_coord(self,frame,frame_d):
        self.dict_init()
        for i in range(8):
            self.get_one_coord(frame,frame_d,i)
        return self.block_loc
        #self.draw(frame, self.block_loc)

    def draw(self,frame,block_loc):
       # cv2.imshow("original",frame)
        for color_index in range(8):
            if block_loc[self.color_name[color_index]] != []:
                cX = block_loc[self.color_name[color_index]][0]
                cY = block_loc[self.color_name[color_index]][1]
                cZ = block_loc[self.color_name[color_index]][2]
                cv2.circle(frame,(cX,cY),2,(0,0,0),-1)
                cv2.putText(frame,"%s Center:%d,%d,%d"%(self.color_name[color_index],cX,cY,cZ),(cX-20,cY-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
        cv2.imshow("labeled img",frame)
           

    def get_one_coord(self,frame,frame_d,color_index):#, color_index):
    #    cv2.namedWindow("Final_whole")
        #_,frame = cap.read()#
     #   print "---------------------",self.color_name[color_index],"----------------------"       
        #frame = freenect.sync_get_video()[0]
        #depth = freenect.sync_get_video()[0]
        #frame = cv2.imread("test1.png")
        #frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        #erosion = cv2.erode(hsv,filter_kernel,iterations = 1)
        mask = cv2.inRange(hsv,self.color_db[color_index][0], self.color_db[color_index][1])
      #  cv2.imshow('hsv inRange',mask)
        # and 
        cv2.bitwise_and(self.board_filter,mask,mask)
      #  cv2.imshow('after board_filter',mask)
        closing = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,self.closing_kernel)
      #  cv2.imshow('cv2.closing',closing)
        #closing = cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.opening_kernel)
        #cv2.imshow('cv2.opening',closing)
        
        cnts = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        count = 0

        for c in cnts:
            try:
                M = cv2.moments(c)
                cX = int(M["m10"]/(M["m00"]))
                cY = int(M["m01"]/(M["m00"]))
                #cv2.drawContours(closing,[c],(0,255,0),2)
                cv2.circle(closing,(cX,cY),2,(0,0,0),-1)
                count += 1
             #   print "count:",count
                area = cv2.contourArea(c)
            #    print "area:",area,
                perimeter = cv2.arcLength(c,True)
            #    print 'perimeter:',perimeter,

                epsilon = 0.1*perimeter
                approx = cv2.approxPolyDP(c,epsilon,True)
                corner_count = len(approx)
             #   print 'approx:',corner_count,'\n'

                depth_coord = np.dot(self.RGB2depth,np.array([[cX],[cY],[1]]))
                cZ = frame_d[int(depth_coord[1])][int(depth_coord[0])]

                #cX_d = int(depth_coord[0])  
                #cY_d = int(depth_coord[1])  
                if (area < self.block_area_max) and (area > self.block_area_min) \
                and (perimeter < self.block_len_max) and (perimeter > self.block_len_min) \
                 and (corner_count < 6) and (cZ < 720) and (cZ > 650):#
                    #(_,_),(_,_),angle = cv2.fitEllipse(c)
                    #level = 1
                   # print "cX,cY:",cX,cY,"depth cX,cY:",cX_d,cY_d
                 #   xyz_3d_camera = np.dot(1/(-0.00307*cZ+3.3309)*self.mat_intr,np.array([cX_d,cY_d,1]))
                    #print "3d camera : ",xyz_3d_camera,cX_d,cY_d,cZ
                    level = 0
                    if (cZ > self.depth_1_min) and (cZ < self.depth_1_max):
                        level = 1
                    elif (cZ > self.depth_2_min) and (cZ < self.depth_2_max):
                        level = 2
                    elif (cZ > self.depth_3_min) and (cZ < self.depth_3_max):
                        level = 3                    
                    #cv2.putText(closing,"%s Center %d:%d,%d,%d"%(self.color_name[color_index],count,cX,cY,cZ),(cX-20,cY-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
                    #print "%s block coordinate:"%(self.color_name[color_index]),cX,cY,cZ
                    angle = 0
                    self.block_loc[self.color_name[color_index]] = [cX,cY,cZ,angle,level]
                    #print [cX,cY,cZ,angle,level]

                    #print "area:",area,'perimeter:',perimeter,'approx:',corner_count,'\n'
                    #print "Block", count, "is what we want"
            except ZeroDivisionError:
                pass
       # print "Total %d blobs detected."%(count)   
      #  cv2.imshow("final",closing)  
        #print "--------------------"

if __name__ == '__main__':
        ex = ImgProcess()