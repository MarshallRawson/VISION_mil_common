#!/usr/bin/env python

from mil_msgs.msg import ObjectInImage
from std_msgs.msg import Header
import cv2
import numpy as np
from mil_msgs.msg import Point2D

class Overlay:
    def __init__(self, header,object_in_image, color = (0,255,0), brush = 3, font = cv2.FONT_HERSHEY_SIMPLEX, font_scale=1, *args, **kwargs):
        
        self.header = header
        self.object = object_in_image
        self.shape = ""
        
        self.color = color
        self.brush = brush
        
        self.font = font
        self.font_scale = font_scale
        
        if len(self.object.points)==0:
            self.shape = "none"
        elif len(self.object.points)==1:
            self.shape = "point"
        elif len(self.object.points)==2:
            self.shape = "rectangle"
        elif len(self.object.points)>2:
            self.shape = "polygon"
        
    def draw_on(self, img):
        if self.shape == "none":
            self.object.points = [None]*1
            self.object.points[0] = Point2D()
            self.object.points[0].x = 0
            self.object.points[0].y = 40
            
            img = self.draw_none(img)
        elif self.shape == "point":
            img = self.draw_point(img)
        elif self.shape == "rectangle":
            img = self.draw_rectangle(img)
        elif self.shape == "polygon":
            img = self.draw_polygon(img)
        
        #TODO: make sure text wont write over other text 
        
        p0 = (int(self.object.points[0].x),int(self.object.points[0].y))
        
        cv2.putText(img,self.object.name,p0, self.font, self.font_scale ,self.color,self.brush,cv2.LINE_AA)
        
        return img
    
    def draw_none(self,img):
        
        return img
    
    def draw_point(self,img):
        
        p = (int(self.object.points[0].x),int(self.object.points[0].y))
        
        img  = cv2.circle(img, p, self.brush, self.color,-1)
        
        return img
    
    def draw_rectangle(self,img):
        p0 = (int(self.object.points[0].x),int(self.object.points[0].y))
        p1 = (int(self.object.points[1].x),int(self.object.points[1].y))
        
        img = cv2.rectangle(img, p0,p1,self.color,self.brush)
        
        return img
    
    def draw_polygon(self,img):
        p = []
        for i in self.object.points:
            p.append([int(i.x),int(i.y)])
        
        pts = np.array(p, np.int32)
        pts = pts.reshape((-1,1,2))
        img = cv2.polylines(img,[pts],True,self.color,self.brush)
        return img




