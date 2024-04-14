# A script to estimate human pose and using PD controller help the turtle bot follow the human
## MANDRED TECH - FUTURE BEYOND OUR STAR

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import numpy as np
from geometry_msgs.msg import Twist
from pose_estimation_movenet_tflite import get_pose
import math
    
class Human_Follower(Node): # Node containing a publisher and subscriber
    def __init__(self):
        super().__init__('human_following_turtlebot_MANDRED_TECH')

        topic_name= '/camera/image_raw'
        self.cmd_vel_publisher=self.create_publisher(Twist,"/cmd_vel",10)
        self.get_logger().info("Control Script Started")

        self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)
        self.subscription 
        self.br = CvBridge()

        self.WIDTH=800
        self.HEIGHT=600
        self.controlX=400
        self.controlY=210

        self.kpl=0.001 # P control for linear velocity
        self.kdl=0.01 # D control for linear velocity
        self.kpr=0.00001 # P control for rotational velocity
        self.kdr=0.001 # D control for rotational velocity


    def img_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame=cv2.resize(current_frame,(self.WIDTH,self.HEIGHT))
        frame,x_cord,y_cord=get_pose(current_frame)
        if(len(x_cord)>0):
            X=sum(x_cord)//len(x_cord)
            Y=sum(y_cord)//len(y_cord)
            cv2.circle(frame, (int(X), int(Y)), 10, (255,255,255), -1) 
            print(X,Y)

            # Calculating linear speed 
            P=euclid_distance(X,Y,self.controlX,self.controlY) * self.kpl
            D=(self.controlY-Y) * self.kdl * -1
            linea=P+D

            # Calculating angular speed
            theta=angle_finder(X,Y,self.controlX,self.controlY,self.controlX,self.controlY,self.WIDTH,self.controlY)
            if theta>=180:
                P_R=(270-theta) * self.kpr
            else:
                P_R=(theta-90) * self.kpr
            D_R=(self.controlX-X) * self.kdr
            rot=D_R + P_R
            
            print("Linear Speed Pred",linea)
            print("Angular Speed Pred",rot)
            self.publish_control(linea,0.0,0.0,0.0,0.0,rot)
            
        else:
            self.publish_control(0.0,0.0,0.0,0.0,0.0,0.0)

        # cv2.line(frame,(0,self.controlY),(self.WIDTH,self.controlY),(0,0,255),2)
        # cv2.line(frame,(self.controlX,0),(self.controlX,self.HEIGHT),(0,0,255),2)
        cv2.imshow("camera", frame)
        cv2.waitKey(1)
    
    def publish_control(self,lx,ly,lz,ax,ay,az): # Publisher function to publish vel to turtlebot vel topic
        cmd=Twist()
        cmd.linear.x=lx
        cmd.linear.y=ly
        cmd.linear.z=lz
        cmd.angular.x=ax
        cmd.angular.y=ay
        cmd.angular.z=az
        self.cmd_vel_publisher.publish(cmd)

def euclid_distance(x1,y1,x2,y2): # Function to calculate the euclidean distance between 2 points
    dist=math.sqrt(((x2-x1)**2)+((y2-y1)**2))
    return dist

def angle_finder(x1,y1,x2,y2,x3,y3,x4,y4): # Function to calculate the angle between two lines
    if (x2-x1)!=0 and (x3-x3)!=0:
        m1=(y2-y1)/(x2-x1)
        m2=(y4-y3)/(x4-x3)
        theta=math.atan2(math.abs(m2-m1)/(1+(m1*m2)))
        return theta
    return 0


def main(args=None):
    rclpy.init(args=args)
    human_follower = Human_Follower()
    rclpy.spin(human_follower)
    human_follower.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()