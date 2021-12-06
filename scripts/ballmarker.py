#!/usr/bin/env python3

import numpy as np
import math
import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class Ball:
	def __init__(self):
		self.t = 0.0
		self.pos = np.array([0.0, 0.5, 0.0])
		### RANDOMIZE LATER
		self.vel = 0.25
		self.xyangle = np.pi / 2 # direction launched in xy plane
		self.projangle = np.pi / 2
		
		
		### CREATE NEW MARKER
		marker = Marker()
		marker.header.frame_id = "world"
		marker.header.stamp = rospy.Time.now()
		marker.id = 100
		marker.ns = "launched_ball"
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
		return marker
		
	
	def pd(self, dt): ### JANKY MATH, FIX
		x = self.vel*np.cos(self.projangle)*np.cos(self.xyangle) * t
		y = self.vel*np.cos(self.projangle)*np.sin(self.xyangle) * t
		z = self.vel*np.sin(self.projangle)*t - 0.5 * 9.8 * np.square(t)
		
		return np.array([x, y, z]).reshape((3,1))

	def vd(self, t):
		x = self.vel*np.sin(self.projangle)*np.cos(self.xyangle)
		y = self.vel*np.sin(self.projangle)*np.sin(self.xyangle)
		z = self.vel*np.cos(self.projangle) - 9.8 * t
		
		return np.array([x,y,z]).reshape((3,1))
		
	def move(self, dt, marker): ### no need to update everything (?)
				
		marker.pose.position.x = self.pos[0]
		marker.pose.position.y = self.pos[1]
		marker.pose.position.z = self.pos[2]
		
		
		self.pos = Ball.pd(self, self.t)
		self.vel = np.linalg.norm(Ball.vd(self, self.t))
		self.t = self.t + dt	
		return marker
	

		

class Generator:
    # Initialize.
	def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.

		self.pub = rospy.Publisher("/visualization_marker_array",
                                         MarkerArray,
                                         queue_size=10, latch=True)
		rospy.sleep(0.25)
		self.array = MarkerArray()
		self.count = 0
		self.MARKERS_MAX = 5
	
		self.timer = 0.0

	
    # Update is called every 10ms!
	
	def update(self, t, dt):
		
		if (self.timer >= 1.0): #### TO FIX
		
			self.timer = 0.0
			newball = Ball.__init__(self)
			self.array.markers.append(newball)
			   # Renumber the marker IDs
			id = 0
			for m in self.array.markers: ### 
				m.id = id
				id += 1
			

		for marker in self.array.markers:
			marker = Ball.move(self, dt, marker)
			
#		if(self.count > self.MARKERS_MAX):
#			self.array.markers.pop(0)



   # Publish the MarkerArray
   
		
		self.pub.publish(self.array)

		self.count += 1
		self.timer = self.timer + dt
		


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
        generator.update(t, dt)


        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

        # Update the time.
        t += dt
