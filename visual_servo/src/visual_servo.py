#!/usr/bin/env python

import cv2
import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import math


class visual_servo():
    def __init__(self):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        self.lowerBound = np.array([0,204,204]) #[16,39,50]
        self.upperBound = np.array([153,255,255])    #[45,255,255]

        self.app = SimpleKeyTeleop()
    
        rospy.Subscriber('vrep/image', Image, self.callback, queue_size=1)
    
        pub = rospy.Publisher('/vrep/laser_switch', Bool, queue_size=1)
        pub.publish("true")
        
        self.bridge = CvBridge()


    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        m_x, m_y, ar = self.image_info(cv_img)
        linear, angular = self.imginfo_movement(m_x, m_y, ar)
        self.app._set_velocity(linear, angular)
        self.app._publish()

    def image_info(self, img): 
        m_x = 0.0
        m_y = 0.0
        area_ratio = 0.0
        counter = 0
        imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV,self.lowerBound,self.upperBound)
        #cv2.imshow("mask",mask)
        #cv2.waitKey(1)
        #cv2.imshow("img",img)
        #cv2.imshow("HSV",imgHSV)
        #cv2.waitKey(1)
        kernelOpen=np.ones((8,8))
        kernelClose=np.ones((25,25))
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        cv2.imshow("maskClose",maskClose)
        cv2.waitKey(1)
	maskFinal=maskClose
	img3,conts,hi =cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        img1=img
        h = 0
        w = 0
    	if conts:
             x,y,w,h=cv2.boundingRect(conts[0])
    	     cv2.rectangle(img1,(x,y),(x+w,y+h),(0,0,255), 2)
             m_x2 = (x+w+x)/2
             m_y2 = (y+y+h)/2
             area_ratio = (float(h*w))/(512*512)
        else:
             m_x2 = 0
             m_y2 = 0
        print("m_x ", m_x2,"\n", "m_y ",m_y2)
        cv2.imshow("img1",img1)
        cv2.waitKey(1)
 
        if area_ratio:
            print("find ball", area_ratio)
        return m_x2, m_y2, area_ratio

    def imginfo_movement(self, m_x, m_y, ar):    
        linear = 0.0
        angular = 0.0
        
        if ar:
          angular = float(20.0*pow((m_x - 256.00)/256,3))
          if ar < 0.1 and angular < 9 and angular >-9: 
            linear = 3
            if angular<0.3:
              angular=0
          elif ar > 0.4:              
            linear = -0.5
          elif ar>0.1 and ar<0.4:
            linear = 0.8
        #print("m_x", m_x, "angular", angular, "linear", linear)
          if ar < 0.001:
            angular = 0
            linear = 0
          print("center pixcel", m_x, "angular", angular,"linear",linear,'rate',ar)
        else: 
            angular = 0
       
        return linear, angular 

class SimpleKeyTeleop():
    def __init__(self):
        self._pub_cmd = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=1)

        self._forward_rate = rospy.get_param('~forward_rate', 8)
        self._backward_rate = rospy.get_param('~backward_rate', 5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._angular = 0
        self._linear = 0


    def run(self):
        self._running = True
        while self._running:
            #self._set_velocity()
            self._publish()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self, linear, angular):
        #if linear > 0:
         #   linear = linear * self._forward_rate
        #else:
         #   linear = linear * self._backward_rate
        #angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)
        #print(self._linear, self._angular, "ok")


if __name__ == '__main__':
    #listener()
    rospy.init_node('visual_servo')
    vs = visual_servo()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

