#!/usr/bin/env python3
#
#   6dof_ikin
#
#   Visualize the 4DOF, moving up/down sinusoidally, using velocity ikin.
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import numpy as np
import math

from sensor_msgs.msg     import JointState
from urdf_parser_py.urdf import Robot

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz

from trajectory import Trajectory
from ballmarker import MarkerPublisher

from splines import Goto, Hold, Goto5



#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.sleep(0.25)

        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        
        # Instantiate the Kinematics
        self.kin = Kinematics(robot, 'world', 'tip')
        self.traj = self.init_traj()
        
                     
        # Initialize starting time t0, lambda, and the joint angle config q
        self.t0    = 0.0
        self.tlag  = 0.0
        self.lmbd  = 10
        self.lmbd_sec = 2
        self.gam   = 0.5
        self.q = np.array([0.0, -0.7, 0.6,  1.3,
                           0.0,  0.9, 0.01, .001 ]).reshape(-1,1) # Holds starting angle
        self.index = 0
    
    def init_traj():
        # resets self.traj
        return
    
    # Return the desired position given time
    def pd(self, t):
        return np.array([0.0, 
                         0.95 - 10.25*math.cos(t), 
                         0.15 + 0.9*math.sin(t)]).reshape(3,1)
    
    # Retun the desired rotation given s
    def Rd(self, t):
        return Rz(np.pi) @ Rx(np.pi/2)
    
    # Return the desired velocity given s, sdot
    def vd(self, t):
        return np.array([0, 
                         10.25*math.sin(t), 
                         0.9*math.cos(t)]).reshape(3,1)
    
    # Return the desired omega given s, sdot
    def wd(self, t):
        return np.array([0,0,0]).reshape(3,1)
    
    # Return the error between two position vectors
    def ep(self, pd, p):
        return pd - p
    
    # Return the error between two rotation matrices
    def eR(self, Rd, R):
        return 0.5*(np.cross(R[:,0:1], Rd[:,0:1], axis=0) +
                    np.cross(R[:,1:2], Rd[:,1:2], axis=0) +
                    np.cross(R[:,2:3], Rd[:,2:3], axis=0))
    def getTraj():
        return self.traj
    
    # Update is called every 10ms!
    def update(self, t):
        # Shut down at time 2pi
        if t > 4*np.pi:
            rospy.signal_shutdown("Done with motion")
            return
            
        # Get time since last update call
        dt = t - self.tlag
        
        # Find the transformation matrix and the inverted jacobian
        (T, J) = self.kin.fkin(self.q)
        
        inrange = -0.06 <= self.q[7] <= 0.06
        
        
        J_inv = np.linalg.inv((J.T @ J) + (np.eye(J.shape[1]) * self.gam**2)) @ J.T
        
        # Get the current position and rotation from the transformation matrix
        pos = p_from_T(T)
        Rm = R_from_T(T)
        
        # Goal velocity is the stack of v and omega
        v_goal = np.vstack((self.vd(t), self.wd(t)))
        # Positional error is the stack of error in position and rotation
        x_til = np.vstack((self.ep(self.pd(t), pos), self.eR(self.Rd(t), Rm)))
        # Get the secondary qdot
        qdot_s = np.array([-self.q[0],
                           -self.q[1] - np.pi/4,
                           -self.q[2],
                           -self.q[3] - np.pi/4,
                           -self.q[4],
                           -self.q[5],
                           -self.q[6], 
                           -self.q[7]]).reshape(-1,1)
                           
                           
        # Compute the new q and qdot with the secondary task
        qdot = J_inv @ (v_goal + self.lmbd*x_til) +\
               ((np.eye(8) - (J_inv @ J)) @ (self.lmbd_sec*qdot_s))
        
        
        
        inrange = -0.06 <= self.q[7] + qdot[7][0][0]*dt <= 0.06
        
        if not inrange:
            J = np.delete(J, -1, axis=1)
            J_inv = np.linalg.inv((J.T @ J) + (np.eye(J.shape[1]) * self.gam**2)) @ J.T
            
            qdot_s = np.delete(qdot_s, -1, axis=0)
            
            qdot = J_inv @ (v_goal + self.lmbd*x_til) +\
                   ((np.eye(7) - (J_inv @ J)) @ (self.lmbd_sec*qdot_s))
            self.q[0:7] += np.array([qdot[x][0][0] for x in range(7)]).reshape(-1,1)*dt
        
        else:
            self.q += np.array([x[0][0] for x in qdot]).reshape(-1,1)*dt # dt * qdot
        
        # Update the current simulation time
        self.tlag = t
        
        # todo track if ball collides
        # reset self.traj on collision
        
        # Create and send the joint state message.
        cmdmsg = JointState()
        cmdmsg.name = ['theta1', 'theta2', 'theta3',
                       'theta4', 'theta5', 'theta6', 
                       'theta7', 'stretch']
        cmdmsg.position = self.q
        cmdmsg.velocity = qdot
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)



#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('sinusoid')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    #traj = Trajectory()

    # Prepare a servo loop at 100Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))
                  
    starttime = rospy.Time.now()
    
    
    
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t = (servotime - starttime).to_sec()

        # Update the controller.
        generator.update(t)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()
