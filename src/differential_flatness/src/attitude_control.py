#!/usr/bin/env python

import numpy as np

import rospy, tf
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command

class Attitude():

    def __init__(self):
        self.mass = rospy.get_param('dynamics/mass')
        self.g = rospy.get_param('dynamics/gravity',9.80665)
        self.thrust_max = rospy.get_param('dynamics/max_F')
        self.thrust_eq = (self.g*self.mass)/(self.thrust_max)

        self.phi = 0.0
        self.theta = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.phi_c = 0.0
        self.theta_c = 0.0
        self.F_c = 0.0
        self.r_c = 0.0

        self.pd = 0
        self.pd_c = 0
        self.w = 0

        self.rot_true = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.prev_time = rospy.get_time()

        # Set Up Publishers and Subscribers
        self.cmd_sub_ = rospy.Subscriber('v_command', Command, self.vcmdCallback, queue_size=5)
        self.hlc_sub_ = rospy.Subscriber('high_level_command', Command, self.hlcCallback, queue_size=5)
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('diff_flat_cmd', Command, queue_size=5, latch=True)

        self.df_cmd = Command()


    def update(self):
        # T_d = self.F_c
        phi_d = self.phi_c
        theta_d = self.theta_c
        r_d = self.r_c
        ct = np.cos(self.theta_c)
        st = np.sin(self.theta_c)
        cp = np.cos(self.phi_c)
        sp = np.sin(self.phi_c)
        rot_des = np.array([cp*st,-sp,cp*ct])

        ctt = np.cos(self.theta)
        stt = np.sin(self.theta)
        cpt = np.cos(self.phi)
        spt= np.sin(self.phi)
        rot_true = np.array([cpt*stt,-spt,cpt*ctt])

        T_correct = np.dot(rot_des,rot_true)

        # self.F_c = self.F_c*T_correct

        #Add pd control to get height to desired height
        # T_d = self.F_c + self.PD(self.pd,self.pd_c,self.w,1,.1
        T_d = self.F_c - self.PD(self.pd,self.pd_c,self.w,.5,1)
        T_d = T_d*T_correct
        # phi_d = self.PD(self.phi,self.phi_c,self.p,7.6394,0.9592)
        # theta_d = self.PD(self.theta,self.theta_c,self.q,7.6394,1.5596)
        # r_d = self.PD(self.r,self.r_c,0,2.8648,0)

        self.df_cmd.x = phi_d
        self.df_cmd.y = theta_d
        #convert thrust to g's *equilibrium thrust to use in roscopter
        self.df_cmd.F = T_d/(self.mass*self.g)*self.thrust_eq#*(.44/self.thrust_eq)
        self.df_cmd.z = r_d

        self.command_pub_.publish(self.df_cmd)

    def PD(self,act,des,derivative,P,D):
        error = des-act
        # differentiator = (2*self.tau-Ts)/(2*tau+Ts)*differentiator + 2/(2*self.tau+Ts)*(error - self.error_d1);
        out = P*error-D*derivative
        return out

    def odometryCallback(self, msg):
        self.t = rospy.get_time()

        self.pd = msg.pose.pose.position.z
        if np.isnan(msg.twist.twist.linear.z):
            self.w = 0.0
        else:
            self.w = msg.twist.twist.linear.z

        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        euler  = tf.transformations.euler_from_quaternion(quat)

        self.phi = euler[0]
        self.theta = euler[1]

        self.p = msg.twist.twist.angular.x
        self.q = msg.twist.twist.angular.y
        self.r = msg.twist.twist.angular.z

    def vcmdCallback(self, msg):
        self.t = rospy.get_time()
        if np.isnan(msg.x):
            self.phi_c = 0
            self.theta_c = 0
        else:
            self.phi_c = msg.x
            self.theta_c = msg.y
        self.F_c = msg.F
        self.r_c = msg.z

    def hlcCallback(self, msg):
        self.t = rospy.get_time()
        self.pd_c = msg.F


def main():
    rospy.init_node('Attitude_control',anonymous=True)

    attitude = Attitude()

    while not rospy.is_shutdown():
        try:
            attitude.update()

        except rospy.ROSInterruptException:
            print "exiting..."
            return

if __name__ == '__main__':
    main()

# if __name__ == '__main__':
#     rospy.init_node('LQR', anonymous=True)
#     try:
#         lqr = LQR()
#     except:
#         rospy.ROSInterruptException
#     pass
