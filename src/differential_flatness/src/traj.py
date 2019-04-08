#!/usr/bin/env python

import numpy as np

import rospy, tf

from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class Trajectory():

    def __init__(self):

        self.alpha = 2.5
        self.beta = 1.25
        self.eta = -2.0
        self.omega = 0.05#2#2.5

        # self.alpha = 3.0
        # self.beta = 3.0
        # self.eta = -1.0
        # self.omega = 2#2.5

        self.takeoff_time = 30
        self.g = rospy.get_param('dynamics/gravity',9.80665)
        self.mass = rospy.get_param('dynamics/mass')

        self.theta_prev = 0.0
        self.theta_dot = 0.0

        self.prev_time = rospy.get_time()

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.traj_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.uff_pub_ = rospy.Publisher('feed_forward_control', Command, queue_size=5, latch=True)
        self.xdes_pub_ = rospy.Publisher('xdes_vel', Command, queue_size=5, latch=True)

        self.cmd = Command()
        self.xdes = Command()
        self.u_ff = Command()

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def odometryCallback(self, msg):
        self.t = rospy.get_time()
        if self.t <= self.takeoff_time:
            if self.t <= self.takeoff_time/5.0:
                pn = self.alpha/self.takeoff_time*self.t*5
                pe = 0
                pd = self.eta/self.takeoff_time*self.t*5
                psi = 0
            else:
                pn = self.alpha
                pe = 0
                pd = self.eta
                psi = 0
            #
            pndot = 0
            pedot = 0
            pddot = 0

            # Feed Forward Controls
            pnddot = 0
            peddot = 0
            pdddot = 0-self.g;
            psidot = 0;

        else:
            # if self.t-self.takeoff_time < 30:
            #     self.omega = 1#2*(self.t-self.takeoff_time)/30
            # Trajectory
            pn = self.alpha*np.cos(self.omega/2.0*(self.t-self.takeoff_time));
            pe = self.beta*np.sin(self.omega*(self.t-self.takeoff_time));
            pd = self.eta;
            psi = 0;

            #
            pndot = -self.alpha*self.omega/2.0*np.sin(self.omega/2.0*(self.t-self.takeoff_time));
            # print pndot
            pedot = self.beta*self.omega*np.cos(self.omega*(self.t-self.takeoff_time));
            pddot = 0;

            # Feed Forward Controls
            pnddot = -self.alpha*self.omega**2.0/4.0*np.cos(self.omega/2.0*(self.t-self.takeoff_time));
            peddot = -self.beta*self.omega**2.0*np.sin(self.omega*(self.t-self.takeoff_time));
            pdddot = 0-self.g;
            psidot = 0;

            # # Trajectory
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

        acc_x = pnddot#/(self.g*self.mass)
        acc_y = peddot#/(self.g*self.mass)
        acc_z = pdddot#/(self.g*self.mass)

        self.cmd.x = pn
        self.cmd.y = pe
        self.cmd.F = pd
        self.cmd.z = psi
        # print pndot
        self.xdes.x = pndot
        self.xdes.y = pedot
        self.xdes.F = pddot
        self.xdes.z = psi

        self.u_ff.x = acc_x
        self.u_ff.y = acc_y
        self.u_ff.F = acc_z
        self.u_ff.z = psidot


        self.u_ff.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE

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
