#!/usr/bin/env python
#TODO add smart filter adjuster

import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ast

from mil_vision_tools import CentroidObjectsTracker, TrackedObject
from vision_utils import centroid
from overlay import Overlay
from mil_msgs.msg import ObjectsInImage, ObjectInImage
import numpy as np

class image_object_tracker:
    def __init__(self):
        
        #Subscribers
        
        
        rospy.Subscriber("/raw_objects_topic", ObjectsInImage,self.objects_in_image_cb)
        
        self.pub_objects_in_image = rospy.Publisher("persistent_objects_topic", ObjectsInImage, queue_size = 1)
        
        self.tracker = CentroidObjectsTracker(expiration_seconds = rospy.get_param("expiration_seconds"), max_distance=rospy.get_param("max_distance"))
        
        
    def objects_in_image_cb(self, unfiltered_objects_in_image):
        # gets unfiltered objects from cv scripts here
        for i in unfiltered_objects_in_image.objects:
            #currently impliments only the centroid tracker, maybe polygon tracker in near future
            c = centroid(i)
            i.attributes = str({"centroid": c})
            #note: data field is holding an ObjectInImage i
            obj = self.tracker.add_observation(unfiltered_objects_in_image.header.stamp, np.array(c), data=i)
        #standard use of CentroidObjectsTracker
        self.tracker.clear_expired(now=unfiltered_objects_in_image.header.stamp)
        
        persistent = self.tracker.get_persistent_objects(min_observations=rospy.get_param("min_observations"), min_age=rospy.Duration(0))
        

        objects_in_image = ObjectsInImage()#these are the actual persisten/filtered objects in image

        objects_in_image.header = unfiltered_objects_in_image.header
        for i in persistent:
            #this is done so that subscriber scripts can have the id of things easily
            #attributes holds a dictionary in string form
            attributes = ast.literal_eval(i.data.attributes)
            attributes["id"] = i.id
            i.data.attributes = str(attributes)
            objects_in_image.objects.append(i.data)
        self.pub_objects_in_image.publish(objects_in_image)

if __name__ == '__main__':
    rospy.init_node("image_object_tracker", anonymous = False)
    image_object_tracker = image_object_tracker()
    rospy.spin()
