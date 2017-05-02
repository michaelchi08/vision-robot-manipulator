import cv2
import numpy as np
from PyQt4 import QtGui, QtCore, Qt
import freenect
import os,shutil

class Video():
    def __init__(self):
        self.currentVideoFrame=np.array([])
        self.currentDepthFrame=np.array([])

        """ 
        Affine Calibration Variables 
        Currently only takes three points: center of arm and two adjacent, 
        corners of the base board. Use more for better calibration.
        Note that OpenCV requires float32 arrays
        """
        self.aff_npoints = 4
        self.real_coord = np.float32([[-305., 305.], [-305.,-305.], [305.,-305.], [305.,305.]])
        self.mouse_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0], [0.0, 0.0]])      
        self.depth_coord = np.float32([[0.0, 0.0],[0.0, 0.0],[0.0, 0.0], [0.0, 0.0]])
        self.mouse_click_id = 0
        self.aff_flag = 0
        self.aff_dep_flag = 0
        #self.aff_matrix = np.float32((2,3))
        #self.aff_d_matrix = np.float32((2,3))

    def affine_transform_diy(self, mouse_coord, real_coord):
        print "affine_transform_diy"
        row_num,_ = mouse_coord.shape
        world_matrix_rearrange = np.zeros((row_num*2,1))
        pixel_matrix_rearrange = np.zeros((row_num*2,6))
        #print mouse_coord,'\n',real_coord,'\n'
        for i in range(row_num*2):
            if i % 2 == 0: #odd line
                pixel_matrix_rearrange[i][2] = 1
                for j in range(2):
                    pixel_matrix_rearrange[i][j] = mouse_coord[i/2][j] # x_pixel
                world_matrix_rearrange[i] = real_coord[i/2][0] # x_world
            else:  # even line
                pixel_matrix_rearrange[i][5] = 1
                for j in range(3,5):
                    pixel_matrix_rearrange[i][j] = mouse_coord[i/2][j-3] # y_pixel
                world_matrix_rearrange[i] = real_coord[i/2][1] # y_world
        #print pixel_matrix_rearrange,'\n',world_matrix_rearrange,'\n'
        ## alternative way
        # world_matrix_rearrange = np.reshape(real_coord, 2*row_num) #b
        # for i in range(row_num):
        #     pixel_matrix_rearrange[2*i] = [mouse_coord[i][0], mouse_coord[1], 1, 0, 0, 0]#A
        #     pixel_matrix_rearrange[2*i+1] = [0, 0, 0, mouse_coord[i][0], mouse_coord[1], 1]
        aff_matrix_diy = np.dot(np.linalg.pinv(pixel_matrix_rearrange),world_matrix_rearrange)
        aff_mat = np.reshape(aff_matrix_diy,[2,3])
        # create file
        var_dir = "./calibration"
        if os.path.isdir(var_dir):
            pass#shutil.rmtree(var_dir)
        else:
            os.mkdir(var_dir)

        with open('./calibration/calibration_matrix.csv','w+') as myfile:
            for row in aff_mat:
                for iter_i in range(len(row)-1):
                    myfile.write(str(row[iter_i]) + ',')
                myfile.write(str(row[-1])+'\n')

        with open('./calibration/board_corner.csv','w+') as myfile:
            for row in mouse_coord:
                print row
                for iter_i in range(len(row)-1):
                    myfile.write(str(row[iter_i]) + ',')
                myfile.write(str(row[-1])+'\n')
        return aff_mat

    def affine_transform_rgb_d_diy(self, mouse_coord, depth_coord):
        print "affine_transform_rgb_d_diy"
        row_num,_ = mouse_coord.shape
        world_matrix_rearrange = np.zeros((row_num*2,1))
        pixel_matrix_rearrange = np.zeros((row_num*2,6))
        for i in range(row_num*2):
            if i % 2 == 0: #odd line
                pixel_matrix_rearrange[i][2] = 1
                for j in range(2):
                    pixel_matrix_rearrange[i][j] = mouse_coord[i/2][j] # x_pixel
                world_matrix_rearrange[i] = depth_coord[i/2][0] # x_world
            else:  # even line
                pixel_matrix_rearrange[i][5] = 1
                for j in range(3,5):
                    pixel_matrix_rearrange[i][j] = mouse_coord[i/2][j-3] # y_pixel
                world_matrix_rearrange[i] = depth_coord[i/2][1] # y_world
        aff_matrix_diy = np.dot(np.linalg.pinv(pixel_matrix_rearrange),world_matrix_rearrange)
        aff_mat = np.reshape(aff_matrix_diy,[2,3])
        # create file
        with open('./calibration/calibration_matrix_depth.csv','w+') as myfile:
            for row in aff_mat:
                for iter_i in range(len(row)-1):
                    myfile.write(str(row[iter_i]) + ',')
                myfile.write(str(row[-1])+'\n')
        return aff_mat


    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        self.currentVideoFrame = freenect.sync_get_video()[0]


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        self.currentDepthFrame = freenect.sync_get_depth()[0]

    def convertFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            img=QtGui.QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QtGui.QImage.Format_RGB888
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to format suitable for QtGui  """
        try:
            DepthFrame = self.currentDepthFrame.copy()
            DepthFrame >>= 2
            DepthFrame = DepthFrame.astype(np.uint8)
            img=QtGui.QImage(DepthFrame,
                             DepthFrame.shape[1],
                             DepthFrame.shape[0],
                             QtGui. QImage.Format_Indexed8
                             )
            return QtGui.QPixmap.fromImage(img)
        except:
            return None


    def loadCalibration(self):
        """
        Load csmera distortion calibration from file and applies to the image:
        Look at camera_cal.py final lines to see how that is done
        This is optional, you do not need to implement distortion correction
        """

        pass

    def blobDetector(self):
        """
        Implement your color blob detector here.  
        You will need to detect 8 different color blobs
        """
        pass
    
    def blockDetector(self):
        """
        Implement your block detector here.  
        You will need to take info from the blog detector and locate
        blocks in 3D space
        """
        pass
