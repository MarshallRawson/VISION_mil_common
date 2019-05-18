#!/usr/bin/env python
import rospy
import tf
from image_geometry import PinholeCameraModel
from mil_vision_tools import MultilateratedTracker
from mil_msgs.msg import ObjectsInImage, ObjectInImage
from mil_ros_tools import Image_Subscriber
from mil_tools import quaternion_matrix
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

import ast
import numpy as np



#still under development
class multilaterator:
    def __init__(self):
        
        rospy.Subscriber("/persistent_objects_topic", ObjectsInImage, self.objects_in_image_cb)
        
        self.pub = rospy.Publisher("/multilaterated_objects", PointStamped, queue_size = 10)
        
        self.tf_listener = tf.TransformListener(rospy.Duration(50))
        
        self.stamp = rospy.Header().stamp
        
        self.tracker = MultilateratedTracker(max_distance= rospy.get_param("multi_max_distance"),
                                             expiration_seconds = rospy.get_param("multi_expiration_seconds"),
                                             min_angle_diff = rospy.get_param("multi_min_angle"))
        
        self.camera = "/camera_topic"
                
        self.camera_model = None
        
        self.image_sub = Image_Subscriber(self.camera)
        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()
         
    
        
    def objects_in_image_cb(self, objects_in_image):
        print len(self.tracker.objects)
        stamp = objects_in_image.header.stamp
        #get the transform for when the objects are from
        if self.camera_model == None:
            return

        try:
            self.tf_listener.waitForTransform(rospy.get_param("map"),
                                              self.camera_model.tfFrame(),
                                              stamp,
                                              rospy.Duration(0.05))
        
        except tf.Exception as e:
            rospy.logwarn(
                 'Could not transform camera to map: {}'.format(e))
            return
        
        (t, rot_q) = self.tf_listener.lookupTransform(
           rospy.get_param("map"), self.camera_model.tfFrame(), stamp)
        
        t = np.asarray(t)
        
        P = np.asarray(self.camera_info.P).reshape(3,4) 
        Pinv =  np.linalg.pinv(P)

        R = quaternion_matrix(rot_q)
 
        for i in objects_in_image.objects:
            #get the id for each the object in image. (these are the same id's from the tracker in iamge_objects_tracker)
            identifier =  ast.literal_eval(i.attributes)['id']
            
            x = ast.literal_eval(i.attributes)['centroid']
            x.append(1)
            v = Pinv.dot(x)[:3]
            v = v/np.linalg.norm(v)#idk if v is already a unit vector here so just to be safe
            #get the relative vector from the camera to the object
            vPrime = R.dot(v)
            #get that vector in map (global) frame
            self.tracker.add_observation(stamp = stamp,features = np.array([t, vPrime]), data = {'id': identifier})
            #add the observation to the tracker. (t is the origin in map and vPrime is the unit vector in map) 
        self.tracker.clear_expired(now = stamp)
       
        persistent = self.tracker.get_persistent_objects(min_observations = rospy.get_param("multi_min_observations"), min_age=rospy.Duration(rospy.get_param("multi_min_age")))#min observations is so log because testing right now, but MUST be at least two for multilateration
        for i in persistent:
            point_stamped = PointStamped()
            point_stamped.header.stamp = stamp
            point_stamped.header.frame_id = rospy.get_param("map")
            point_stamped.point.x = i.data['pose'][0] 
            point_stamped.point.y = i.data['pose'][1]
            point_stamped.point.z = i.data['pose'][2]

            self.pub.publish(point_stamped)
        
        return
        

if __name__ == '__main__':
    rospy.init_node('multilaterator', anonymous = False)
    mulitlaterator = multilaterator()
    rospy.spin()
