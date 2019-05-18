#!/usr/bin/env python


import rospy
from mil_msgs.msg import ObjectsInImage, ObjectInImage, Point2D
from sensor_msgs.msg import Image
import cv2
from math import cos, sin, pi
from random import shuffle, randint

class ex_cv:
    def __init__(self):
        rospy.Subscriber('/camera/front/right/image_raw', Image,self.image_cb)
        self.pub=rospy.Publisher('VISION',ObjectsInImage,queue_size=1)
        self.objects_in_image = ObjectsInImage()
        
        #recipricol chance that each shape will be detected in any given frame...
        #ie: 30 = 1/30 chance
        
        
        self.box_chance = rospy.get_param("box")
        self.spot_chance = rospy.get_param("spot")
        self.strange_chance = rospy.get_param("strange")
        self.in_here_somewhere_chance = rospy.get_param("in_here_somewhere")
        
        self.width = 1920
        self.height = 1080
        
        self.center = [(self.width/2), (self.height/2)]
        self.r = 100
        
        
    def image_cb(self, msg):
        #print('ex_cv:   got image')
        self.objects_in_image.header = msg.header
        box = ObjectInImage()
        
        box.name = "box"
        
        box.points = [None]*2
        
        box.points[0]= Point2D()
        box.points[0].x = msg.width/3
        box.points[0].y = msg.height/3
        
        box.points[1]= Point2D()
        box.points[1].x = (msg.width*2)/3
        box.points[1].y = (msg.height*2)/3
        
        
        spot = ObjectInImage()
        
        spot.name = "spot"
        
        spot.points = [None]*1
        
        spot.points[0]= Point2D()
        spot.points[0].x = (msg.width*4)/5
        spot.points[0].y = (msg.height*4)/5
        
        
        strange = ObjectInImage()
        
        strange.name = "strange"
        
        strange.points = [None]*9
        
        self.center[0] += randint(-25,25)
        self.center[1] += randint(-25,25)
        
        theta=0
        i=0
        self.r += randint(-10,10)
        while i<len(strange.points):
            strange.points[i]=Point2D()
            strange.points[i].x = (self.r* cos(theta))+self.center[0]
            strange.points[i].y = (self.r* sin(theta))+self.center[1]
            
            theta = theta+((pi*2)/len(strange.points))
            i = i+1
        
        in_here_somewhere = ObjectInImage()
        
        in_here_somewhere.name = "in here somewhere"
        
        if randint(1,self.box_chance) == 1:
            self.objects_in_image.objects.append(box)
        
        if randint(1,self.spot_chance) == 1:
            self.objects_in_image.objects.append(spot)
        
        if randint(1,self.strange_chance) == 1:
            self.objects_in_image.objects.append(strange)
        
        if randint(1,self.in_here_somewhere_chance) == 1:
            self.objects_in_image.objects.append(in_here_somewhere)
        
        #self.objects_in_image.objects = [box, spot, strange, in_here_somewhere]
        
        #print len(self.objects_in_image.objects)
        
        shuffle (self.objects_in_image.objects)
        self.pub.publish(self.objects_in_image)
        
        self.objects_in_image.objects = []

if __name__ == '__main__':
    rospy.init_node('ex_cv', anonymous=True)
    ex_cv = ex_cv()
    rospy.spin()




