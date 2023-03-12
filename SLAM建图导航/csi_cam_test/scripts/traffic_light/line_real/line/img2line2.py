#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Float32MultiArray, Int16
from sensor_msgs.msg import Image as imgemsg
from _41combine import *
from Timer import *
import numpy as np
from sensor_msgs.msg import CompressedImage
from collections import deque

que = deque()
for i in range(0, 5):
    que.append(0)


class find_line:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("line_control_img", imgemsg, queue_size=1)
        self.control_pub = rospy.Publisher("line_number_direction", Float32MultiArray, queue_size=1)
        self.bridge = CvBridge()
        self.start_line_sub = rospy.Subscriber("cv_control", Int16, self.callback_start)
        self.image_sub = rospy.Subscriber("img_compressed", CompressedImage, self.callback, queue_size=1,
                                          buff_size=2 ** 24)
        # a[0]=1start contol follow line
        self.array = [1, 1]
        # arr[0]=1,run
        self.start_line = 1
        self.control_dir = 0
        self.count_lowret = 0

        self.all_direct = [0, 0, 0, 0, 0]
        self.counting = 0
        self.q = que

    def callback_start(self, data):
        print('judge line start or not')
        cv_con = data.data
        print(cv_con)
        # except ValueError as e:
        # print(e)
        if cv_con == 2:
            self.start_line = 1
            self.array[0] = 1
            print('open line')
        if cv_con == 3:
            self.start_line = 1
            self.array[0] = 1
            print('close line')

    # def callback_start(self, data):
    #     try:
    #         print('judge line start or not')
    #         cv_con = data
    #     except ValueError as e:
    #         print(e)
    #     if cv_con == 2:
    #         self.start_line = 1
    #         self.array[0] = 1
    #         print('open line')
    #     if cv_con == 3:
    #         self.start_line = 0
    #         self.array[0] = 0
    #         print('close line')

    def callback(self, data):
        print('go into callbackline', self.start_line)
        if self.start_line == 1:
            time_all = timer(6)
            # time_msgcv = timer(5)
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # time_msgcv('on linux commsg_cv')
            print("Processing frame | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())

            # time_line = timer(6)
            # cv2.imshow('sub-img', cv_image)
            # cv2.waitKey(2)

            cv_image = cv2.resize(cv_image, (640, 480))
            control_direction, out_net, ret, err = follow_line(cv_image, 0.75)
            ## the  smaller 
            if -8<control_direction<8:
                control_direction=1.5*control_direction
            control_direction=control_direction*0.32
            # collect the direct and change the direct
            self.counting = self.counting + 1
            a = self.counting % 5

            if err == 0:
                self.all_direct[a] = control_direction

            # refined the line---------
            # if err == 1:
            #     print('lossing the line,try to find--', end='')
            #     if self.all_direct[a - 1] >= self.all_direct[a - 2] >= self.all_direct[a - 3]:
            #         print('right not enough---')
            #         control_direction = self.all_direct[a - 1] * 1.2
            #     elif self.all_direct[a - 1] <= self.all_direct[a - 2] <= self.all_direct[a - 3]:
            #         print('left not enogh----')
            #         control_direction = self.all_direct[a - 1] * 1.2
            #     else:
            #         print('just follow last')
            #         control_direction = self.all_direct[a - 1]
            #     self.all_direct[a] = self.all_direct[a - 1]
            if err == 1:
                print(self.q)
                print('losing the line,try to find--', end='')
                if self.q[4] > self.q[3] >= self.q[2] > 0:
                    print('right not enough---')
                    control_direction = +100
                    # cv2.waitKey(0)
                elif self.q[4] < self.q[3] <= self.q[2] < 0:
                    print('left not enough----')
                    # control_direction = self.all_direct[a - 1] - 50
                    control_direction = -100
                    # cv2.waitKey(0)
                else:  # 设计为左边偏移
                    control_direction = control_direction

            # fail to follow line, complete judge
            self.count_lowret = max(0, self.count_lowret)
            if err == 1:
                self.count_lowret = self.count_lowret + 1
            if err == 0:
                self.count_lowret = self.count_lowret - 0.3
            if ret > 80:
                self.count_lowret = 0
            if ret > 65:
                self.count_lowret = self.count_lowret - 1
            print('==ret===========================', ret, '===count======', self.count_lowret)
            if self.count_lowret > 20:
                # self.array[0] = 0
                # self.start_line = 0
                self.count_lowret = 0
                print('not find line ,kill line_info-------------------------------------------------------')

            # change the quene
            self.q.popleft()
            control_di_int = int(control_direction)
            self.q.append(control_di_int)

            # publish data
            self.control_dir = control_direction
            self.array[1] = control_direction

            control_pubnum = Float32MultiArray(data=self.array)
            # time_line('time_line_all')

            time_show = timer(5)
            out_net = cv2.resize(out_net, (120, 90))
            cv2.imshow("outnet window", out_net)
            cv2.waitKey(1)
            time_show('time_show')
            # 再将opencv格式额数据转换成ros image格式的数据发布
            print('type:', type(out_net))

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(out_net, 'mono8'))
                self.control_pub.publish(control_pubnum)
                time_all('all thing finish------')
                print("Processing pub the control | Delay:%6.3f" % (rospy.Time.now() - data.header.stamp).to_sec())
            except CvBridgeError as e:
                print(e)

        if self.start_line == 0:
            print('the line off--')
            self.array[0] = 0
            self.array[1] = 0
            control_pubnum = Float32MultiArray(data=self.array)
            self.control_pub.publish(control_pubnum)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("find_line")
        rospy.loginfo("Starting find_line node")
        find_line()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down find line node.")
        cv2.destroyAllWindows()
