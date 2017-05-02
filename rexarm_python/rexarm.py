import lcm
import time
import numpy as np

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t

PI = np.pi
D2R = PI/180.0
ANGLE_TOL = 2*PI/180.0 
R2D = 180.0/3.141592

""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ Commanded Values """
        self.num_joints = 7
        self.joint_angles = [0.0] * self.num_joints # radians
        # you must change this to control each joint speed separately 
        self.speed_list = [0.5] * self.num_joints        # 0 to 1
        self.max_torque = 0.5                    # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0

        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("ARM_STATUS",
                                        self.feedback_handler)
        self.msg_len = self.num_joints
        # D-H kinematics parameters
        self.theta_mat = [0,np.pi/2,0,0]
        self.alpha_mat = [np.pi/2,0,0,0]
        self.d_mat = [117.78, 0,0,0,0]
        self.a_mat = [0,101,101,111]
        # swap inverse kinematic parameter
        self.l1_swap = 117.7
        self.l2_swap = 101
        self.l3_swap = 101
        self.l4_swap = 72.5
       # self.arm_range = self.l2_swap + self.l3_swap + self.l4_swap

        self.l_front_arm = 46 #######need measure
        self.dist_from_gripper_to_block = 42
        self.l5 = self.l_front_arm + self.dist_from_gripper_to_block

        self.angle_dim = len(self.theta_mat)
        self.start_point = np.zeros([4,1])
        self.start_point[3] = 1
        self.theta1_limit = [-np.pi,np.pi]
        self.theta2_limit = [-2.20,2.158]
        self.theta3_limit = [-2.19,2.12]
        self.theta4_limit = [-1.85,1.93]
        # still need to change
        self.theta5_limit = [-np.pi,np.pi]
        self.theta6_limit = [-np.pi,np.pi]
        self.theta7_limit = [-1.4,-0.2]

        self.tol_check = 0.25###########################need tune
        self.grab_load = 0.25###########################need tune

        self.grab_angle = -1.2
        self.drop_angle = -0.33#0.45

       # self.drop_load = 0.1# no need for now

        self.angle_grab_tol = 0.1
        self.angle_drop_tol = 0.1

        self.base_rotate_tol = 0.05 # for 3 
        self.wrist_rotate_tol = 0.1

        #self.method_1_offset = 20
        ## cmd directly to worker
        self.tol_check_cmd = 0.05#0.13

    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM. 
        You need to activelly call this function to command the arm.
        You can uncomment the print statement to check commanded values.
        """    
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        angle_check = self.clamp(self.joint_angles)
        if angle_check == 1:
            for i in range(msg.len):
                cmd = dynamixel_command_t()
                cmd.utime = int(time.time() * 1e6)
                cmd.position_radians = self.joint_angles[i]
                # you SHOULD change this to contorl each joint speed separately 
                cmd.speed = self.speed_list[i]
                cmd.max_torque = self.max_torque
                print cmd.speed,cmd.max_torque
                #print "radians:",cmd.position_radians
                msg.commands.append(cmd)
            self.lc.publish("ARM_COMMAND",msg.encode())
        else:
            print "Angle check not passed."
            print np.rad2deg(self.joint_angles)


    def get_feedback(self):
        """
        LCM Handler function
        Called continuously from the GUI 
        """
        self.lc.handle_timeout(50)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature


    def clamp(self,angle_check):
        """
        Clamp Function
        Limit the commanded joint angles to ones physically possible so the 
        arm is not damaged.
        LAB TASK: IMPLEMENT A CLAMP FUNCTION
        """
        if (angle_check[0]<self.theta1_limit[0]) or (angle_check[0]>self.theta1_limit[1]):
            #print "theta1 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        if (angle_check[1]<self.theta2_limit[0]) or (angle_check[1]>self.theta2_limit[1]):
           # print "theta2 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        if (angle_check[2]<self.theta3_limit[0]) or (angle_check[2]>self.theta3_limit[1]):
           # print "theta3 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        if (angle_check[3]<self.theta4_limit[0]) or (angle_check[3]>self.theta4_limit[1]):
           # print "theta4 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        if (angle_check[4]<self.theta5_limit[0]) or (angle_check[4]>self.theta5_limit[1]):
           # print "theta5 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        if (angle_check[5]<self.theta6_limit[0]) or (angle_check[5]>self.theta6_limit[1]):
            #print "theta6:",angle_check[5]<self.theta6_limit[0],angle_check[5]>self.theta6_limit[1],angle_check[5]
            print "theta6 not qualify."
            return 0#self.joint_angles[0] = 180*D2R
        return 1

    def plan_command(self):
        """ Command planned waypoints """
        pass

    def rexarm_FK(self,input_angles):
        """
        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """
        trans_mat = np.eye(4)

        for i in range(self.angle_dim):
            #if i!=self.angle_dim-2:
            delta = input_angles[i]

            theta = self.theta_mat[i] + delta
            alpha = self.alpha_mat[i]
            d = self.d_mat[i] 
            a = self.a_mat[i] 
            temp = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],\
                            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],\
                            [0, np.sin(alpha), np.cos(alpha), d],\
                            [0, 0, 0, 1]])
            trans_mat = np.dot(trans_mat,temp)
            if i == self.angle_dim - 2:
                trans_mat_2ed_last = trans_mat
        end_point = np.dot(trans_mat, self.start_point)
        end_point_2ed_last = np.dot(trans_mat_2ed_last, self.start_point)
        z_temp = end_point[2] - end_point_2ed_last[2]
        xy_temp = np.sqrt(end_point[0]**2+end_point[1]**2)-\
                np.sqrt(end_point_2ed_last[0]**2+end_point_2ed_last[1]**2)
        theta_end = np.arctan2(z_temp, xy_temp)
        
    	return end_point+theta_end#np.concatenate((end_point[0:2], theta_end), axis = 1)
        
    def rexarm_IK(self,pose,method):
        """
        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        ###### input is the fourth motor location point
        l1 = self.l1_swap
        l2 = self.l2_swap
        l3 = self.l3_swap
        l4 = self.l4_swap
        motor_angle_cmd = []

        if method == 1:
            x_e = pose[0]
            y_e = pose[1]
            z_e = pose[2]
            angle = pose[3]

            l_bar_1_square = (z_e-l1)**2+x_e**2+y_e**2
            l_bar_1 = np.sqrt(l_bar_1_square)
            if (l_bar_1> l2+l3) or (l2>l_bar_1+l3) or (l3>l_bar_1+l2):
                print "Method %s : Error (IK): l_bar_1,l2,l3: "%(method),l_bar_1,l2,l3
                return 0
            theta1 = np.arctan2(y_e, x_e)
            # convert theratical theta to command theta
            theta1_cmd = theta1 + np.pi/2 
            theta1_cmd = (theta1_cmd+np.pi)%(2*np.pi)-np.pi
            #theta3 by default we use elbow up, so the value don't need to change
            theta3 = np.pi - np.arccos((l2**2+l3**2-l_bar_1_square)/(2*l2*l3))
       #     print "theta3:",np.rad2deg(theta3)
            alpha_2 = np.arccos((l2**2+l_bar_1_square-l3**2)/(2*l2*l_bar_1))
            theta2 = np.pi/2 \
            - alpha_2\
            - np.arctan2(z_e-l1, np.sqrt(x_e**2+y_e**2))
             # this because of swap method
            theta4 = np.pi - theta2 - theta3
            theta4 = (theta4+np.pi)%(2*np.pi)-np.pi
            # orientation
            theta5 = np.pi/2 - angle - theta1
            theta5_cmd = (theta5+np.pi)%(2*np.pi)-np.pi
            # No.6 gripper wrist
            theta6 = 0
            # convert theta2~5 just change negative/positive sign
            motor_angle_cmd.append(theta1_cmd)
            motor_angle_cmd.append(-theta2)
            motor_angle_cmd.append(-theta3)
            motor_angle_cmd.append(-theta4)
            motor_angle_cmd.append(-theta5_cmd) # swap joint, pick joint joint_angles[5] set default,
            motor_angle_cmd.append(theta6)
            if self.clamp(motor_angle_cmd) == 1:
                return motor_angle_cmd
            else:
                return 0 

        elif method == 2:
            x_e = pose[0]
            y_e = pose[1]
            z_e = pose[2]
            angle = pose[3]

            l_bar_1_square = (z_e-l1)**2+x_e**2+y_e**2
            l_bar_1 = np.sqrt(l_bar_1_square)
            if (l_bar_1> l2+l3) or (l2>l_bar_1+l3) or (l3>l_bar_1+l2):
                print "Method %s : Error (IK): l_bar_1,l2,l3: "%(method),l_bar_1,l2,l3
                return 0
            theta1 = np.arctan2(y_e,x_e) + np.pi/2#-np.arctan2(x_e,y_e)
            theta3 = np.arccos((-l2**2-l3**2+l_bar_1_square)/(2*l2*l3))
            alpha_2 = np.arccos((l2**2+l_bar_1_square-l3**2)/(2*l2*l_bar_1))
            theta2 = np.arctan2(np.sqrt(x_e**2+y_e**2), z_e - self.l1_swap)- alpha_2
            theta4 = theta3 + theta2 #(np.pi)/2 - theta3 - theta2
            #theta6 = angle - (-theta1)
            #theta6 = (theta6+np.pi)%(2*np.pi)-np.pi
            theta6 = 0
            theta5 = 0
            motor_angle_cmd.append(theta1)
            motor_angle_cmd.append(-theta2)
            motor_angle_cmd.append(-theta3)
            motor_angle_cmd.append(theta4)
            motor_angle_cmd.append(theta5) # swap joint, pick joint joint_angles[5] set default,
            motor_angle_cmd.append(theta6)

            if self.clamp(motor_angle_cmd) == 1:
                return motor_angle_cmd
            else:
                return 0 
        elif method == 3:
            x = pose[0]
            y = pose[1]
            z = pose[2]
            theta = pose[3]
            theta6_condition = pose[4]
            
            theta1 = np.arctan2(y, x)
            theta1_cmd = theta1 + np.pi/2 
            theta1_cmd = (theta1_cmd+np.pi)%(2*np.pi)-np.pi
            if theta6_condition == 0:
                theta6 = 0
            elif theta6_condition == 1:
                theta6 = np.pi/2
            elif theta6_condition == 2:
                theta6 = theta1
            ## joint 4
            x4 = x+self.l5*np.sin(theta)*np.cos(theta1)-l4*np.cos(theta)*np.cos(theta1)
            y4 = y+self.l5*np.sin(theta)*np.sin(theta1)-l4*np.cos(theta)*np.sin(theta1)
            z4 = z+self.l5*np.cos(theta)+l4*np.sin(theta)
            #print "joint 4 coordinates:",x4,y4,z4
            l_bar_1_square = x4**2+y4**2+(z4-l1)**2
            l_bar_1 = np.sqrt(l_bar_1_square)
            if (l_bar_1< l2+l3) and (l2<l_bar_1+l3) and (l3<l_bar_1+l2):
                alpha_2 = np.arccos((l2**2+l_bar_1_square-l3**2)/(2*l2*l_bar_1))
                alpha_3 = np.arccos((l3**2+l_bar_1_square-l2**2)/(2*l_bar_1*l3))
                alpha_4 = np.arctan2((z4-l1),np.sqrt(x4**2+y4**2))
                theta3 = np.pi - np.arccos((l2**2+l3**2-l_bar_1_square)/(2*l2*l3))#alpha_2 + alpha_3
                theta3_cmd = (theta3+np.pi)%(2*np.pi)-np.pi
                theta2 = np.pi/2 -alpha_2 - alpha_4
                theta5 = 0# swap angle
                # theta7 is gripper
                theta4 = theta + alpha_4 - alpha_3

                motor_angle_cmd.append(theta1_cmd)
                motor_angle_cmd.append(-theta2)
                motor_angle_cmd.append(-theta3_cmd)
                motor_angle_cmd.append(-theta4)
                motor_angle_cmd.append(-theta5) 
                motor_angle_cmd.append(theta6)
                if self.clamp(motor_angle_cmd) == 1:
                    #print "degree:",np.rad2deg(motor_angle_cmd)
                    return motor_angle_cmd
                else:
                    return 0
            else:
                #print " Error (IK): l_bar_1,l2,l3"
                return 0
 
    def rexarm_collision_check(self,q):
        """
        Perform a collision check with the ground and the base
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 
