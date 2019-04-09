#!/usr/bin/env python

import numpy as np

import rospy, tf

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
import scipy.linalg


class LQR():

    def __init__(self):

        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.pndot = 0.0
        self.pedot = 0.0
        self.pddot = 0.0

        self.pn_d = 0.0
        self.pe_d = 0.0
        self.pd_d = 0.0
        self.psi_d = 0.0

        self.pndot_d = 0.0
        self.pedot_d = 0.0
        self.pddot_d = 0.0

        self.pnddot_c = 0.0
        self.peddot_c = 0.0
        self.pdddot_c = 0.0
        self.r_c = 0.0
        #x = [Pn, Pe, Pd, dPn, dPe, dPd, psi]
        #u = [ddPn, ddPe, ddPd, dPsi]
        #y = [Pn,Pe,Pd,psi]
        A = np.array([[0,0,0,1,0,0,0],
                      [0,0,0,0,1,0,0],
                      [0,0,0,0,0,1,0],
                      [0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0]])

        B = np.array([[0,0,0,0],
                      [0,0,0,0],
                      [0,0,0,0],
                      [1,0,0,0],
                      [0,1,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

        b = np.array([0,0,0,0,0,1,0])
        b = b.T
        C = np.array([[1,0,0,0,0,0,0],
                      [0,1,0,0,0,0,0],
                      [0,0,1,0,0,0,0],
                      [0,0,0,0,0,0,1]])
        #from brysons rule Qii = 1/max allowable state
        # Rii = 1/max allowable control
        
        # Q = 0.1*np.matmul(C.T,C)
        # R = 12*np.eye(4)

        # Q = np.diag([.01,.01,.1,0,0,0,0.1])
        # R = np.diag([1000,1000,1000,12])
        
        Q = np.diag([1/1.0,1/1.0,1/1.0,0,0,0,1/10.0])
        R = np.diag([1/0.01,1/0.01,1/0.001,1/0.1])


        # Q = np.diag([10,10,10,10,10,10,0.1])
        # R = np.diag([1000,1000,1000,12])
        self.K,S,E = self.computeLQR(A,B,Q,R)
        # print ('%.3f' % self.K)
        # self.K = np.array([[1.0,0,0,1.4142,0,0,0],
                           # [0,1.0,0,0,1.4142,0,0],
                           # [0,0,1.0,0,0,1.4142,0],
                           # [0,0,0  ,0,0,0     ,1]])

        self.prev_time = rospy.get_time()

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.cmd_sub_ = rospy.Subscriber('high_level_command', Command, self.hlcCallback, queue_size=5)
        self.uff_sub_ = rospy.Subscriber('feed_forward_control', Command, self.uffCallback, queue_size=5)
        self.xdes_sub_ = rospy.Subscriber('xdes_vel', Command, self.xdesCallback, queue_size=5)
        self.is_flying_sub_ = rospy.Subscriber('is_flying',Bool,self.isFlyingCallback,queue_size = 5)
        self.command_pub_ = rospy.Publisher('u_raw', Command, queue_size=5, latch=True)

        self.u_raw = Command()
        self.u_raw.mode = Command.MODE_XACC_YACC_YAWRATE_AZ
        self.flying = False
        # while not rospy.is_shutdown():
        #     # wait for new messages and call the callback when they arrive
        #     rospy.spin()

    def computeLQR(self,A,B,Q,R):
        """Solve the continuous time lqr controller.
        dx/dt = A x + B u
        cost = integral x.T*Q*x + u.T*R*u
        """
        #ref Bertsekas, p.151 
        #first, try to solve the ricatti equation
        S = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
         
        #compute the LQR gain
        K = np.matrix(np.linalg.inv(R)*(B.T*S))
         
        eigVals, eigVecs = np.linalg.eig(A-B*K)
         
        return np.array(K), np.array(S), eigVals

    def update(self):
        X = np.array([self.pn,self.pe,self.pd,self.pndot,self.pedot,self.pddot,self.psi])
        X_des = np.array([self.pn_d,self.pe_d,self.pd_d,self.pndot_d,self.pedot_d,self.pddot_d,self.psi_d,])
        # print X
        X_tilde = X-X_des

        u_tilde = np.matmul(-self.K,X_tilde)

        u_ff = np.array([self.pnddot_c,self.peddot_c,self.pdddot_c,self.r_c])

        u = u_tilde+u_ff

        self.u_raw.x = u[0]
        self.u_raw.y = u[1]
        self.u_raw.F = u[2]
        self.u_raw.z = u[3]

        if self.flying == True:
            self.command_pub_.publish(self.u_raw)

    def isFlyingCallback(self,msg):
        self.flying = msg.data
        print (self.flying)

    def odometryCallback(self, msg):
        self.t = rospy.get_time()
        self.pn = msg.pose.pose.position.x
        self.pe = msg.pose.pose.position.y
        self.pd = msg.pose.pose.position.z

        if np.isnan(msg.twist.twist.linear.x):
            self.u = 0.0
            self.v = 0.0
            self.w = 0.0
        else:
            self.u = msg.twist.twist.linear.x
            self.v = msg.twist.twist.linear.y
            self.w = msg.twist.twist.linear.z

        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        euler  = tf.transformations.euler_from_quaternion(quat)

        self.phi = euler[0]
        self.theta = euler[1]
        self.psi = euler[2]

        cp = np.cos(self.phi);
        sp = np.sin(self.phi);
        ct = np.cos(self.theta);
        st = np.sin(self.theta);
        tt = np.tan(self.theta);
        cs = np.cos(self.psi);
        ss = np.sin(self.psi);

         # translational kinematics model
        tkm = np.array([[ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss],
                        [ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs],
                        [-st  , sp*ct         , cp*ct]])

        uvw = np.array([[self.u],[self.v],[self.w]])

        neddot = np.matmul(tkm,uvw)

        self.pndot = neddot[0]
        self.pndot = neddot[1]
        self.pndot = neddot[2]

        if np.isnan(msg.twist.twist.angular.x):
            self.p = 0.0
            self.q = 0.0
            self.r = 0.0
        else:
            self.p = msg.twist.twist.angular.x
            self.q = msg.twist.twist.angular.y
            self.r = msg.twist.twist.angular.z


    def hlcCallback(self, msg):
        self.t = rospy.get_time()
        self.pn_d = msg.x
        self.pe_d = msg.y
        self.pd_d = msg.F
        self.psi_d = msg.z

    def uffCallback(self, msg):
        self.t = rospy.get_time()
        self.pnddot_c = msg.x
        self.peddot_c = msg.y
        self.pdddot_c = msg.F
        self.r_c = msg.z

    def xdesCallback(self, msg):
        self.t = rospy.get_time()
        self.pndot_d = msg.x
        self.pedot_d = msg.y
        self.pddot_d = msg.F

def main():
    rospy.init_node('LQR_control',anonymous=True)

    lqr = LQR()

    while not rospy.is_shutdown():
        try:
            lqr.update()

        except rospy.ROSInterruptException:
            print ("exiting...")
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
