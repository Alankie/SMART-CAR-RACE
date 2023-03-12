#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, Float32MultiArray, Int16, UInt8
import nav_msgs.msg
import tf
import math


class vision_control:
    
    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.z1 = 0

        # self.reply=0

        # self.light_x_max=rospy.get_param("light_x_max",3.61)  #红绿灯模块开启x坐标    hotel
        # self.light_x_min=rospy.get_param("light_x_min",1.91)  #红绿灯模块结束x坐标
        # self.light_y_max=rospy.get_param("light_y_max",0.5)  #红绿灯模块开启y坐标
        # self.light_y_min=rospy.get_param("light_y_min",-1)  #红绿灯模块结束y坐标

        
        self.light_x_max=rospy.get_param("light_x_max",2.72)  #红绿灯模块开启x坐标
        self.light_x_min=rospy.get_param("light_x_min",1.79)  #红绿灯模块结束x坐标
        self.light_y_max=rospy.get_param("light_y_max",-4.87)  #红绿灯模块开启y坐标
        self.light_y_min=rospy.get_param("light_y_min",-6.35)  #红绿灯模块结束y坐标

        # self.linex1=rospy.get_param("linex1",1.7)
        # self.line_x_min1=rospy.get_param("line_x_min1",3.70)    #车道线启动区间   hotel
        # self.line_x_max1=rospy.get_param("line_x_max1",3.10)
        # # self.liney1=rospy.get_param("liney1",-1.1)
        # self.line_y_min1=rospy.get_param("line_y_min1",-1)
        # self.line_y_max1=rospy.get_param("line_y_max1",0.5)

        self.line_x_min1=rospy.get_param("line_x_min1",0.47)    #车道线启动区间
        self.line_x_max1=rospy.get_param("line_x_max1",1.34)
        # self.liney1=rospy.get_param("liney1",-1.1)
        self.line_y_min1=rospy.get_param("line_y_min1",-4.25)
        self.line_y_max1=rospy.get_param("line_y_max1",-3.70)

        # self.line_x_min2=rospy.get_param("line_x_min2",4.60)   #车道线停止区间   hotel
        # self.line_x_max2=rospy.get_param("line_x_max2",4.50)
        # self.line_y_min2=rospy.get_param("line_y_min2",-1)
        # self.line_y_max2=rospy.get_param("line_y_max2",0.5)
        
        self.line_x_min2=rospy.get_param("line_x_min2",-2.70)   #车道线停止区间
        self.line_x_max2=rospy.get_param("line_x_max2",0.71)
        self.line_y_min2=rospy.get_param("line_y_min2",-1.76)#-1.76
        self.line_y_max2=rospy.get_param("line_y_max2",0.80)

        self.line_x_max3=-1.00  #车道线出来后的直行区
        self.line_x_min3=-1.80
        self.line_y_max3=-1.27
        self.line_y_min3=-1.76


        self.c1x=rospy.get_param("c1x",-0.262)
        self.c1y=rospy.get_param("c1y",-3.78)
        self.r1=rospy.get_param('r1',1.20)
        self.w1=rospy.get_param('w1',0.405)

        self.c2x=rospy.get_param("c2x",-0.262)
        self.c2y=rospy.get_param("c2y",-2.58)
        self.r2=rospy.get_param('r2',1.20)
        self.w2=rospy.get_param('w2',0.405)

        #订阅/odom
        self.odom_msg = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        # self.cv_return=rospy.Subscriber("",Float32MultiArray,self.cv_return_callback)
        #发布cv_control
        self.line_ = rospy.Subscriber('dispatcher/line_start',UInt8, self.line_callback) 
        self.control = rospy.Publisher("cv_control",Int16, queue_size=1)
        # self.control_bringup = rospy.Publisher("bringup_control",Int16,queue_size=1)

        self.timer=rospy.Timer(rospy.Duration(0.025),self.timer_callback)
        self.cv_mode=0

        self.line_begin=0
        # self.bringup_mode=0
        
    def line_callback(self,msg):

        if msg.data==1:
            rospy.loginfo("received line start command")
            self.cv_mode=2
            self.line_begin=True
    def odom_callback(self,msg):
        self.x1=msg.pose.pose.position.x
        self.y1=msg.pose.pose.position.y
        self.z1=msg.pose.pose.position.z

        # print "x1=",self.x1,"\ny1=",self.y1,"\nz1=",self.z1
        #print "light_x_max=",self.light_x_max
        #print "light_x_min=",self.light_x_min
        #print "light_y_max=",self.light_y_max
        #print "light_y_min=",self.light_y_min



        if(self.x1<=self.light_x_max and self.x1>=self.light_x_min and self.y1<=self.light_y_max and self.y1>=self.light_y_min):        #红绿灯启动
            self.cv_mode=1
            rospy.loginfo("LIGHT")

        #elif(self.x1>=self.line_x_min1 and self.x1<=self.line_x_max1 and self.y1>=self.line_y_min1 and self.y1<=self.line_y_max1):     #车道线启动
        elif self.line_begin:
            rospy.loginfo("LINE_START")
            self.cv_mode=2



        elif(self.x1>=self.line_x_min2 and self.x1<=self.line_x_max2 and self.y1>=self.line_y_min2 and self.y1<=self.line_y_max2):      #车道线停止
        
            self.cv_mode=3
            if(self.x1>=self.line_x_min3 and self.x1<=self.line_x_max3 and self.y1>=self.line_y_min3 and self.y1<=self.line_y_max3):
                self.cv_mode=4
                # print("直走")
            else:
                pass
            

            rospy.loginfo("LINE_END")


            # self.bringup_mode=1  #跟随导航
        else:                                   #啥都没有
            if not self.line_begin:
                self.cv_mode=0
            # print "6666"
       
        
        
        # self.control_bringup(self.bringup_mode)

    # def cv_return_callback(self,backmsg):
    #     self.reply= backmsg[0]

    def timer_callback(self,event):
        #rospy.loginfo('callback')
        #print("cv_mode=",self.cv_mode)
        self.control.publish(self.cv_mode)
        if self.cv_mode==2:
            self.line_begin=False
            self.cv_mode=0
        #print('x=',self.x1,'y=',self.y1)


    def is_in_circles(self):
        start_area=False
        if(self.x1>=self.linex2 and self.x1<=self.linex3 and self.y1>=self.liney2 and self.y1<=self.liney3):
	    start_area=True
        out_of_circle_1=True
        out_of_circle_2=True
        dist1=math.sqrt((self.x1-self.c1x)**2+(self.y1-self.c1y)**2)
        dx1=self.x1-self.c1x
        dy1=self.y1-self.c1y
        if(dist1<(self.r1-self.w1/2) or dist1>self.r1+self.w1/2 or dx1>0 or dy1<0):
            out_of_circle_1=True
            rospy.loginfo('out of circle 1')
        else:
            rospy.loginfo('in circle 1')
            out_of_circle_1=False
        dist2=math.sqrt((self.x1-self.c2x)**2+(self.y1-self.c2y)**2)
        dx2=self.x1-self.c2x
        dy2=self.y1-self.c2y
        if(dist2<(self.r2-self.w2/2) or dist2>self.r2+self.w2/2 or dx2<0 or dy1>0):
            out_of_circle_2=True
            rospy.loginfo('in circle2')
            rospy.loginfo('out of circle 2')        
        else:
            out_of_circle_2=False
            print('d1=',dist1,'d2=',dist2)
            print('d1max=',self.r2+self.w2/2)
            print('d2max=',self.r2+self.w2/2)
        if out_of_circle_1 and out_of_circle_2 and not start_area:
            return True
        else:
            return False


if __name__=="__main__":
    rospy.init_node("vision_controller", anonymous=True)
    rospy.loginfo("start vision_controller mode")
    

    con=vision_control()
    
    
    rospy.spin()
