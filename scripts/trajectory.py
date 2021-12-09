#!/usr/bin/env python3
#


import math
import numpy as np
import rospy


class Trajectory:
    def __init__(self, velocity, angle, xyangle):
        # velocity: number
        # angle: launch angle, 
        # xyangle: direction launched (xy plane)
        
        self.pos = np.array([0.0, 2.0, 0.0])
        self.vel = velocity
        self.angle = angle
        self.xyangle = xyangle
        self.grav = 2.0
    
    	# todo -- define path in terms of t (or just set params)
        return
        
    def getPath(self, t):
        vx = self.vel*np.cos(self.angle)*np.cos(self.xyangle)
        vy = self.vel*np.cos(self.angle)*np.sin(self.xyangle)
        vz = self.vel*np.sin(self.angle) - self.grav * t
#        vz = self.vel*np.sin(self.angle)
        
        
        
        
        # todo -- get p of the ball for a time t 
        print (np.array([self.pos[0]+vx*t, self.pos[1]+vy*t, self.pos[2]+vz*t - 0.5*self.grav*t**2]))
        
        return np.array([self.pos[0]+vx*t, 
                         self.pos[1]+vy*t, 
                         self.pos[2]+vz*t - 0.5*self.grav*t**2])



# TO TEST TRAJECTORIES
#

#if __name__ == "__main__":
#    # Prepare/initialize this node.
#    rospy.init_node('traj')

#    # Instantiate the trajectory generator object, encapsulating all
#    # the computation and local variables.
#    traj = Trajectory(5,45,0)

#    # Prepare a servo loop at 100Hz.
#    rate  = 100;
#    servo = rospy.Rate(rate)
#    dt    = servo.sleep_dur.to_sec()
#    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
#                  (dt, rate))


#    # Run the servo loop until shutdown (killed or ctrl-C'ed).
#    t = 0
#    while not rospy.is_shutdown():

#        # Update the controller.
#            
#        print(traj.getPath(t))


#        # Wait for the next turn.  The timing is determined by the
#        # above definition of servo.
#        servo.sleep()

#        # Update the time.
#        t += dt


