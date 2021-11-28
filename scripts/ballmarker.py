#!/usr/bin/env python3

import numpy as np
import math
import rospy

from visualization_msgs.msg import Marker

#from abc import ABCMeta, abstractmethod
#import math

#import tf.broadcaster
#import rospy
#from visualization_msgs.msg import Marker
#from geometry_msgs.msg import PoseStamped


class Generator:
    # Initialize.
	def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.

#        topic = 'visualization_marker_array'
#        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
#        rosp.sleep(0.25)
#        markerArray = MarkerArray()
#        count = 0
#        MARKERS_MAX = 100
        
#		tf_broadcaster = tf.broadcaster.TransformBroadcaster()
		self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
		self.pos = np.array([0.0, 0.5, 0.0])
        
        ### INPUT VELOCITY AND ANGLE HERE FROM GUI
		self.vel = 0.25
		self.xyangle = np.pi / 2 # direction launched in xy plane

	def pd(self, t): ### JANKY MATH, FIX
		x = self.vel*np.cos(t)*np.cos(self.xyangle)
		y = self.vel*np.cos(t)*np.sin(self.xyangle)
		z = self.vel*np.sin(t) - 0.5 * 9.8 * np.square(t)
		
#		print(self.vel)
#		print(self.xyangle)
#		
#		print(np.array([x,y,z]))

		return np.array([x, y, z]).reshape((3,1))
    	
	
	def vd(self, t):
		x = self.vel*np.sin(t)*np.cos(self.xyangle)
		y = self.vel*np.sin(t)*np.sin(self.xyangle)
		z = self.vel*np.cos(t)
		
		return np.array([x,y,z]).reshape((3,1))
		
	def reset_check(self, t):
		if (self.pos[2] < 0.0):
		
			print('in here')
			self.pos = np.array([0.0, 0.5, 0.0])
			
			### CHECK NEW GIVEN PARAMS
			self.vel = 0.25
			self.xyangle = np.pi / 2
			print(self.pos)
	
    # Update is called every 10ms!
	
	def update(self, t):
		
		## ORIENTATION DOESNT MATTER - IGNORE FOR NOW
	#    quat_mat = T.quaternion_matrix(quat_goal)
	#    quat_goal = T.quaternion_from_matrix(quat_mat)
	#    tf_broadcaster.sendTransform(tuple(pos_goal), (quat_goal[1],quat_goal[2],quat_goal[3],quat_goal[0]), rospy.Time.now(), 'ik_goal', 'common_world')

		self.reset_check(t)
		
		marker = Marker()
		marker.header.frame_id = 'common_world'
		marker.id = 100
		marker.ns = 'launched_ball'
		marker.type = marker.SPHERE
		marker.action = marker.ADD					
		marker.pose.position.x = self.pos[0]
		marker.pose.position.y = self.pos[1]
		marker.pose.position.z = self.pos[2]
		
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		
		marker.scale.x = .05
		marker.scale.y = .05
		marker.scale.z = .05
		marker.color.a = 1
		marker.color.r = 0.0
		marker.color.g = 0.9
		marker.color.b = 0.2
		self.pub.publish(marker)
		
		self.pos = self.pd(t)
		self.vel = np.linalg.norm(self.vd(t))
		
		print(self.pos)


#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('ballmarker')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    t = 0
    while not rospy.is_shutdown():

        # Update the controller.
        generator.update(t)


        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

        # Update the time.
        t += dt
