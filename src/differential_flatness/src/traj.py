#!/usr/bin/env python

'''
traj.py
Generate a desired trajectory and take the first and second derivatives
of the trajectory to derive feed forward control and desired
state values at each time step
'''

import numpy as np
import rospy, tf
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class Trajectory():

    def __init__(self):

        # Parameters for feed forward trajectory

        self.alpha = 2.5 #amplitude of pn wave
        self.beta = 1.25 #amplitude of pe wave
        self.eta = -2.0 #pd
        self.omega_f = 1.5 #period of cycle final
        self.omega_s = 0.05 #period of cycle start


        # self.alpha = 3.0
        # self.beta = 3.0
        # self.eta = -1.0
        # self.omega = 2#2.5

        self.takeoff_time = 10#30.0 #how long before the trajectory begins(s)
        self.g = rospy.get_param('dynamics/gravity',9.80665)
        self.mass = rospy.get_param('dynamics/mass')

        self.theta_prev = 0.0
        self.theta_dot = 0.0

        self.prev_time = rospy.get_time()
        self.start_time = self.prev_time

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.traj_pub_ = rospy.Publisher('trajectory', Command, queue_size=5, latch=True)
        self.uff_pub_ = rospy.Publisher('u_ff', Command, queue_size=5, latch=True)
        self.xdes_pub_ = rospy.Publisher('xdes_vel', Command, queue_size=5, latch=True)

        self.cmd = Command()
        self.xdes = Command()
        self.u_ff = Command()

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def alphaBlend(self,spinup_time):
        #this function is used to slowly ramp up to desired period in trajectory
        alpha = min(1,(self.t-self.takeoff_time)/spinup_time)
        omega = (1-alpha)*self.omega_s + alpha*self.omega_f
        return omega

    def odometryCallback(self, msg):
        self.t = rospy.get_time()-self.start_time

        if self.t <= self.takeoff_time:
            # first get into position to start the feed forward trajectory
            #override mode value to tell controller how to handle u_ff

            #also set up a roll in so that velocity is correct when trajectory starts
            rollin = 2 * self.takeoff_time/10
            amp = self.beta*self.alpha
            omega_roll = np.pi/(2*rollin)
            offset = -amp/omega_roll

            # # to turn off roll in
            # rollin = 0
            # amp = 0
            # omega_roll = 0
            # offset = 0

            self.u_ff.mode = 0 #this says to just use PID to get into position
            if self.t <= self.takeoff_time/5.0:
                # to avoid a step input in the beginning, this creates a ramp
                # up to the step to get it into position for trajectory
                pn = self.alpha#/self.takeoff_time*self.t*5
                pe = offset
                pd = self.eta/self.takeoff_time*self.t*5
                psi = 0
                pedot = 0
                peddot = 0
            elif self.t <= (self.takeoff_time-rollin):
                pn = self.alpha
                pe = offset
                pd = self.eta
                psi = 0
                pedot = 0
                peddot = 0
            else:
                # this is where roll in is implemented
                self.uff_mode = 1
                pn = self.alpha
                pe = offset*np.cos(omega_roll*(self.t-self.takeoff_time+rollin))
                pd = self.eta
                psi = 0
                pedot = amp*np.sin(omega_roll*(self.t-self.takeoff_time+rollin))
                peddot = omega_roll*amp*np.cos(omega_roll*(self.t-self.takeoff_time+rollin))
            #
            pndot = 0

            pddot = 0

            # Feed Forward Controls
            pnddot = 0

            pdddot = 0-self.g;
            psidot = 0;

        else:
            self.u_ff.mode = 1
            # self.omega = self.alphaBlend(60.0)
            self.omega = self.alphaBlend(0.05)

            # Trajectory
            # the following are for a figure 8 trajectory
            pn = self.alpha*np.cos(self.omega/2.0*(self.t-self.takeoff_time));
            pe = self.beta*np.sin(self.omega*(self.t-self.takeoff_time));
            pd = self.eta;
            psi = 0;

            # Velocities (derivatives of trajectory)
            pndot = -self.alpha*self.omega/2.0*np.sin(self.omega/2.0*(self.t-self.takeoff_time));
            pedot = self.beta*self.omega*np.cos(self.omega*(self.t-self.takeoff_time));
            pddot = 0;
            psidot = 0;

            # Accelerations (derivatives of velocities
            pnddot = -self.alpha*self.omega**2.0/4.0*np.cos(self.omega/2.0*(self.t-self.takeoff_time));
            peddot = -self.beta*self.omega**2.0*np.sin(self.omega*(self.t-self.takeoff_time));
            pdddot = 0-self.g;
            psiddot = 0

            # # Trajectory
            # # the following are for a circular/oval trajectory
            # pn = self.alpha*np.cos(self.omega*(self.t-self.takeoff_time));
            # pe = self.beta*np.sin(self.omega*(self.t-self.takeoff_time));
            # pd = self.eta;
            # psi = 0;
            #
            # #
            # pndot = -self.alpha*self.omega*np.sin(self.omega*(self.t-self.takeoff_time));
            # # print pndot
            # pedot = self.beta*self.omega*np.cos(self.omega*(self.t-self.takeoff_time));
            # pddot = 0;
            #
            # # Feed Forward Controls
            # pnddot = -self.alpha*self.omega**2.0*np.cos(self.omega*(self.t-self.takeoff_time));
            # peddot = -self.beta*self.omega**2.0*np.sin(self.omega*(self.t-self.takeoff_time));
            # pdddot = 0-self.g;
            # psidot = 0;

        acc_x = pnddot
        acc_y = peddot
        acc_z = pdddot

        #desired position (from trajectory)
        self.cmd.x = pn
        self.cmd.y = pe
        self.cmd.F = pd
        self.cmd.z = psi

        #deisred velocities
        self.xdes.x = pndot
        self.xdes.y = pedot
        self.xdes.F = pddot
        self.xdes.z = psi

        #feed forward controls
        self.u_ff.x = acc_x
        self.u_ff.y = acc_y
        self.u_ff.F = acc_z
        self.u_ff.z = psidot

        # self.u_ff.mode = Command.MODE_XACC_YACC_YAWRATE_AZ
        self.xdes.mode = Command.MODE_PASS_THROUGH
        self.cmd.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE

        self.traj_pub_.publish(self.cmd)
        self.uff_pub_.publish(self.u_ff)
        self.xdes_pub_.publish(self.xdes)


if __name__ == '__main__':
    rospy.init_node('Trajectory', anonymous=True)
    try:
        traj = Trajectory()
    except:
        rospy.ROSInterruptException
    pass
