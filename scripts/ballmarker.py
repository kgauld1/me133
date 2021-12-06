#!/usr/bin/env python3
#
#   ballmarker.py
#
#   Publish:    /visualization_marker_array     visualization_msgs/MarkerArray
#
import sys
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#  Marker Publisher
#
#  This continually publishes the marker array.
#
class MarkerPublisher:
    def __init__(self, p0):
        # Prepare the publishers (latching for new subscribers).
        self.pub_mark  = rospy.Publisher("/visualization_marker_array",
                                         MarkerArray,
                                         queue_size=1, latch=True)

        # Create the sphere marker.
        self.marker = Marker()
        self.marker.header.frame_id    = "world"
        self.marker.header.stamp       = rospy.Time.now()
        self.marker.action             = Marker.ADD
        self.marker.ns                 = "point"
        self.marker.id                 = 1
        self.marker.type               = Marker.SPHERE
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x    = p0[0]
        self.marker.pose.position.y    = p0[1]
        self.marker.pose.position.z    = p0[2]
        self.marker.scale.x            = .05
        self.marker.scale.y            = .05
        self.marker.scale.z            = .05
        self.marker.color.a            = 1
        self.marker.color.r            = 0.0
        self.marker.color.g            = 0.9
        self.marker.color.b            = 0.2


        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)

    def update(self, p):
        self.marker.pose.position.x    = p[0]
        self.marker.pose.position.y    = p[1]
        self.marker.pose.position.z    = p[2]

    def publish(self):
        # Publish.
        now = rospy.Time.now()
        self.marker.header.stamp = now
        self.pub_mark.publish(self.mark)

    def loop(self):
        # Prepare a servo loop at 10Hz.
        servo = rospy.Rate(10)
        rospy.loginfo("Point-Publisher publication thread running at 10Hz...")

        # Loop: Publish and sleep
        while not rospy.is_shutdown():
            self.publish()
            servo.sleep()

        # Report the cleanup.
        rospy.loginfo("Point-Publisher publication thread ending...")

class Generator:
    # Initialize.
    def __init__(self):
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.count = 0
        self.timer = 0 

        self.p0 = [0, 0.2, 0.2]
        self.ppub = MarkerPublisher(self.p0)

    
    # Update is called every 10ms!
    def update(self, t, dt): 
        self.ppub.publish()
#        self.ppub.update([0, 0.2, np.sin(t)])
        self.ppub.update([0, np.sin(t), np.sin(t)])

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
