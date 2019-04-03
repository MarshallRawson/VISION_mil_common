#!/usr/bin/env python
from mil_msgs.msg import ObjectInImage




def centroid(object_in_image):
    x=0
    y=0
    if len(object_in_image.points) != 0:
        n=0
        for i in object_in_image.points:
            x+=i.x
            y+=i.y
            n+=1
        x = int(x/n)
        y = int(y/n)
    return [x,y]
    
    
    
    