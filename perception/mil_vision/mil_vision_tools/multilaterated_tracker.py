#!/usr/bin/env python
import abc
import numpy as np

from objects_tracker import ObjectsTracker, TrackedObject
import rospy
import math

class MultilateratedTracker(ObjectsTracker): 
    '''
    Implements ObjectsTracker, using the distance between the closet approach between projection vectors 

    Features must be added as a (2,3) numpy array ([x,y,z],[x,y,z]) format:origin,unit vector in map refrence frame
                                                     /\       /\
                                                   origin  unit vector
    '''
    def __init__(self, max_distance= .5, expiration_seconds = 10, min_angle_diff = 0.5,**kwargs):
        super(MultilateratedTracker, self).__init__(expiration_seconds=expiration_seconds, max_distance=max_distance, **kwargs)
        self.min_angle_diff = min_angle_diff
        #note: data is a dictionary in these obejcts
    def add_observation(self, stamp, features, data=None):
        '''
        Add a new observation to the tracked objects.
        @stamp: rospy.Time object representing the time this observation arrived
        @return: The id of the object, or -1 if it was added
        '''
        for obj in self.objects:           
            dist_of_approach, pose =  self.distance(obj.features, features)
            if dist_of_approach != None and dist_of_approach<self.max_distance:
                #if angle between the two vectos is large enough and they approach is close
                obj.data.update({'pose':pose})
                obj.data.update({'id':data['id']})
                #give this object the id of the most recent vector (the old one may have gone away)
                obj.update(stamp, features, data = obj.data)
                return obj
            
            if dist_of_approach==None and obj.data['id']==data['id']:
                #if the angle is too small, but the ids are the same, do nothing
                return
            
        # No match found, add new
        new_obj = TrackedObject(self.max_id, stamp, features, data=data)
        self.max_id += 1
        self.objects.append(new_obj)
        return new_obj
   
    
    def distance(self, a, b):
        #print "testing distance"
        p0 = np.array(a[0])
        p1 = np.array(b[0])
	
        v0 = a[1]
        v1 = b[1]
        
        angle = math.acos(np.clip(np.dot(v0,v1),-1.0,1.0))
        
        if angle<self.min_angle_diff:
         #print "      angle too small"
         return None, None

        print "angle:", angle
        p2 = p1-p0
        
        v2 = np.cross(v1,v0)
        
        V = np.transpose(np.array([v0,-v1,v2]))
        T = np.linalg.inv(V).dot(p2)

        if T[0]<0 or T[1] <0:
            return None, None 

        p0Prime = p0+(T[0]*v0)
        p1Prime = p1+(T[1]*v1)
        
        dist = np.linalg.norm(p0Prime-p1Prime) 
        pose = np.true_divide(np.add(p0Prime, p1Prime), 2)
        print "dist of approach:", dist
        print "pose:", pose
        '''
        Math:
        v's are 3d vectors
        p's are 3d points(origins in this case)
        t's are scalars(held in T)
        L's are lines
        L0 = p0+t0*v0
        L1 = p1+t1*v1
        the vector component of the shortest line between them is v2 = v1 x v0

        Let L2 be the line connecting L0 and L1 at their closest points of approching each other
        the origin of L2 is some point on L0: p0+t0*v0

        therfore: L2 = p0+t0*v0+t2*v2, also written later as: L2 = p2+t2*v2
       
        since some point on L2 will be on L1 by definition,
        it follows that
        p0+t0*v0+t2*v2 = p1+t1*v1
        
        t0*v0 - (t1*v1) + t2*v2 = p1-p0

        written in matrix form:
        --         --
        |v0x v1x v2x|
        |v0y v1y v2y| = V
        |v0z v1z v2z|
        --         --
         --
        |t0|
        |t1| = T
        |t2|
         --
         ---
        |p2x|
        |p2y| = p2
        |p2z|
         ---
         --       --     --     ---
        |v0x v1x v2x|   |t0|   |p2x|
        |v0y v1y v2y| x |t1| = |p2y|
        |v0z v1z v2z|   |t2|   |p2z|
         --       --     --     ---
        
        T = inv(V)*p2

        the closest points on L0 and L1 occur at L0(t0) = p0Prime and L1(t1) = p1Prime
        
        
        '''
        return dist, pose
    
    def get_persistent_objects(self, min_observations=10, min_age=rospy.Duration(0)):
        '''
        Get a list of objects which have 
            persisted sufficiently long
            have enough observations
            have a pose(have had an angle that is sufficiently large'
        
        @param min_observations: Minimum number of times the object was seen to be returned
        @param min_age: Minimum age an object must be to be returned in result
        @return: List of TrackedObject instances of those objects meeting the above criteria
        '''
        return filter(lambda obj: obj.age >= min_age and obj.observations >= min_observations and 'pose' in obj.data, self.objects)





 
