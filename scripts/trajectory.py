#!/usr/bin/env python3
#
#   splines.py
#
#   TO IMPORT, ADD TO YOUR CODE:
#   from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
#
#
#   This library provides the cubic and quintic spline trajectory
#   segment classes.  I seperated from the main code, just so I don't
#   have to copy/paste it again and again....
#
#   The classes are:
#
#      seg = CubicSpline(p0, v0, pf, vf, T, space='Joint')
#      seg = Goto(       p0,     pf,     T, space='Joint')
#      seg = Hold(       p,              T, space='Joint')
#      seg = Stay(       p,                 space='Joint')
#
#      seg = QuinticSpline(p0, v0, a0, pf, vf, af, T, space='Joint')
#      seg = Goto5(        p0,         pf,         T, space='Joint')
#
#   Each spline object then provides a
#
#      (pos, vel) = seg.evaluate(t)
#      T          = seg.duration()
#      space      = seg.space()
#
#   That is, each segment can compute the position and velocity for
#   the specified time.  Note it can also report which "space" it
#   wants, but inthe end it is the user's responsibility to
#   interpret/use the pos/vel information accordingly.
#
#   The segments also assume a t=0 start time.  That is, when calling
#   evaluate(), please first subtract the start time of the segment!
#
#   The p0,pf,v0,vf,a0,af may be NumPy arrays.
#
import math


class Trajectory:
    def __init__(self, params_for_motion):
    	# todo -- define path in terms of t (or just set params)
        return
    def getPath(t):
        # todo -- get p of the ball for a time t 
        return 





class Linear:
    def __init__(self, p0, pf, T, space='Joint'):
        self.p0 = p0
        self.T = T
        self.vel = (pf - p0)/T
        self.usespace = space
    
    # Return the segment's space
    def space(self):
        return self.usespace

    # Report the segment's duration (time length).
    def duration(self):
        return self.T
    
    def evaluate(self, t):
        p = self.p0 + self.vel * t
        return (p, self.vel)

