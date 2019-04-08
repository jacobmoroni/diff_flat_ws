#!/usr/bin/env python

import numpy as np

import rospy, tf
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command

class Inverse():

    def __init__(self):
        self.mass = rospy.get_param('dynamics/mass')
        self.g = rospy.get_param('dynamics/gravity',9.80665)
        self.thrust_max = rospy.get_param('dynamics/max_F')
        self.thrust_eq = (self.g*self.mass)/(self.thrust_max)

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.pnddot_c = 0.0
        self.peddot_c = 0.0
        self.pdddot_c = 0.0
        self.r_c = 0.0

        self.prev_time = rospy.get_time()
        self.t = 0
        # Set Up Publishers and Subscribers
        # self.cmd_sub_ = rospy.Subscriber('u_raw', Command, self.urawCallback, queue_size=5)
        self.cmd_sub_ = rospy.Subscriber('feed_forward_control', Command, self.urawCallback, queue_size=5)
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('v_command', Command, queue_size=5, latch=True)

        self.v_command = Command()

    def saturate(self,x,min,max):
        if x > max:
            x = max
        elif x < min:
            x = min
        return x


    def update(self):

        rot_psi = np.array([[np.cos(self.psi), -np.sin(self.psi), 0],
                            [np.sin(self.psi),  np.cos(self.psi), 0],
                            [0               ,0                 , 1]])
        T_d = self.mass*np.sqrt(self.pnddot_c*self.pnddot_c+self.peddot_c*self.peddot_c+self.pdddot_c*self.pdddot_c)

        z = np.matmul(rot_psi,np.array([[float(self.pnddot_c)],[float(self.peddot_c)],[float(self.pdddot_c)]]))*self.mass/(-T_d)

        phi_d = np.arcsin(float(-z[1]))
        theta_d = np.arctan2(z[0],z[2])
        r_d = self.r_c*np.cos(self.theta)*np.cos(self.phi)-self.q*np.sin(self.phi)

        self.v_command.x = phi_d
        self.v_command.y = theta_d
        self.v_command.F = T_d
        self.v_command.z = r_d

        self.command_pub_.publish(self.v_command)


    def odometryCallback(self, msg):
        self.t = rospy.get_time()

        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        euler  = tf.transformations.euler_from_quaternion(quat)

        self.phi = euler[0]
        self.theta = euler[1]
        self.psi = euler[2]

        if np.isnan(msg.twist.twist.angular.x):
            self.p = 0.0
            self.q = 0.0
            self.r = 0.0
        else:
            self.p = msg.twist.twist.angular.x
            self.q = msg.twist.twist.angular.y
            self.r = msg.twist.twist.angular.z

    def urawCallback(self, msg):
        self.t = rospy.get_time()
        self.pnddot_c = msg.x
        self.peddot_c = msg.y
        self.pdddot_c = msg.F
        self.r_c = msg.z

def main():
    rospy.init_node('Inverse_control',anonymous=True)

    inverse = Inverse()

    while not rospy.is_shutdown():
        try:
            inverse.update()

        except rospy.ROSInterruptException:
            print "exiting..."
            return

if __name__ == '__main__':
    main()
