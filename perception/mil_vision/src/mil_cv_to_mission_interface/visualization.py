#!/usr/bin/env python

import rospy

from mil_msgs.msg import ObjectsInImage
from mil_msgs.msg import ObjectInImage
#from mil_msgs import Point2D

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from overlay import Overlay
from random import randint
import sys
import ast
import Queue

class visualization:
    def __init__(self, disco = False, minus = [], only = [],**kwargs):
        
        assert not (only != [] and minus !=[])
        
        #Subscribers
        rospy.Subscriber(rospy.get_param("camera_feed"), Image,self.image_cb)
        
        rospy.Subscriber('persistent_objects_in_image', ObjectsInImage, self.tracked_objects_cb)
        
        #self.image = Image()
        self.overlays = []
        self.bridge = CvBridge()
        
        self.disco = disco
        
        self.colors = self.color_gen(64)
        
        self.minus = minus
        self.only = only

        self.image_buffer = Queue.Queue()
        
    def tracked_objects_cb(self, tracked_objects):
        
        msg=self.image_buffer.get()
        #get most recent image
        if self.image_buffer.qsize() > 200:#give warning if the jank image_buffer is ver big, something may have frozen
            rospy.logwarn("visualization buffer is running %d frames behind camera topic (normal frames behind seen in testing is ~100)" % self.image_buffer.qsize())
        while (not self.image_buffer.empty()) and (tracked_objects.header.stamp < msg.header.stamp):#we are only interested in the image that is from the same time as the objects
            msg=self.image_buffer.get()
        #if we have gone through all of the images and none of them were satisfactory, return
        if self.image_buffer.empty():
            rospy.logwarn("visualization image buffer is empty")#this almost never issues
            return
        self.overlays = [Overlay(header = tracked_objects.header, object_in_image = i) for i in tracked_objects.objects]#create an array of overlays of every object published
        overlays = self.select_overlays(msg, self.overlays)#filter these overlays based on the flags passed in when launched
        
        #make a cv image out of the rosmsg
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        #draw on all the selected overlays
        cv_image = self.draw_on(cv_image, overlays)
        #show the cv image in an iamge window
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)

    def image_cb(self, msg): 
        self.image_buffer.put(msg)
        #add image to queue
        
    def draw_on(self, img, overlays=[]):
        #kind of redundent
        for i in overlays:
            img = i.draw_on(img)
        return img
    
    def select_overlays(self,msg, all_overlays):
        
        selected_overlays = all_overlays
        
        if self.minus!=[]:
            for i in all_overlays:
                if (i.object.name in self.minus):
                    selected_overlays.remove(i)
        
        elif self.only !=[]:
            for i in all_overlays:
                if (i.object.name not in self.only):
                    selected_overlays.remove(i)
        
        selected_overlays.sort(key = lambda x: x.object.name)
        
        #the overlays are also given non-default colros here
        
        if self.disco == True:
            self.colors = []
            self.colors = self.color_gen(len(selected_overlays))
        i =0
        for j in selected_overlays:
            j.color = self.colors[i]
            i+=1
            if i>=len(self.colors):
                i=0
        return selected_overlays
    
    def color_gen(self, n):
        #generate random colors to draw the overlays with to make more visible
        c = []
        for i in range(n):
            c.append((randint(0,255),randint(0,255),randint(0,255)))
        return c



if __name__ == '__main__':
    rospy.init_node('visualization', anonymous=False)
    
    kwargs = {}
    
    kwargs.update({'disco':rospy.get_param("disco")})
    
    
    minus = rospy.get_param("minus").split(",")
    
    for i in minus:
        if i == '':
            minus.remove(i)
    
    kwargs.update({'minus':minus})
    
    only = rospy.get_param("only").split(",")
    
    for i in only:
        if i == '':
            only.remove(i)
    kwargs.update({'only':only})
    
    
    visualization = visualization(**kwargs)
    rospy.spin()
    cv2.destroyAllWindows()

