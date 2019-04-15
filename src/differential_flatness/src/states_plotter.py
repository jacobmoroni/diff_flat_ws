#!/usr/bin/env python
import rospy
import time, tf
import numpy as np
import pyqtgraph as pg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from rosflight_msgs.msg import Command
import scipy.io as sio
import rospkg

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self):
        # get parameters from server
        # self.outfile = diff_flat_data.mat
        rospack = rospkg.RosPack()
        self.folder = rospack.get_path('differential_flatness')
        self.file = rospy.get_param('~file_name', 'temp.mat')

        self.t_win = rospy.get_param('~time_window', 5.0)
        self.time0 = 0
        self.init_time = True
        self.time_t = 0

        # setup subsribers
        # rospy.Subscriber('/ekf_estimate', Odometry, self.estimateCallback)
        rospy.Subscriber('ground_truth/odometry/NED', Odometry, self.truthCallback)
        rospy.Subscriber('trajectory', Command, self.trajecoryCallback)
        rospy.Subscriber('xdes_vel', Command, self.xdesCallback)
        rospy.Subscriber('diff_flat_cmd', Command, self.uffCallback)
        rospy.Subscriber('command', Command, self.cmdCallback)
        rospy.Timer(rospy.Duration(2), self.savecallback)


        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])
        self.w = pg.GraphicsWindow(title='States vs Time')
        self.w.resize(1200,1000)

        # initialize plots in one window
        self.p_pn = self.w.addPlot()
        self.p_pn.addLegend(size=(1,1), offset=(1,1))
        self.p_pe = self.w.addPlot()
        self.p_pd = self.w.addPlot()
        self.w.nextRow()
        self.p_phi = self.w.addPlot()
        self.p_theta = self.w.addPlot()
        self.p_psi = self.w.addPlot()
        self.w.nextRow()
        self.p_u = self.w.addPlot()
        self.p_v = self.w.addPlot()
        self.p_w = self.w.addPlot()
        self.w.nextRow()
        self.p_p = self.w.addPlot()
        self.p_q = self.w.addPlot()
        self.p_r = self.w.addPlot()



        # label the plots
        self.p_pn.setLabel('left', 'pn')
        self.p_pe.setLabel('left', 'pe')
        self.p_pd.setLabel('left', 'pd')
        self.p_phi.setLabel('left', 'phi')
        self.p_theta.setLabel('left', 'theta')
        self.p_psi.setLabel('left', 'psi')
        self.p_u.setLabel('left', 'u')
        self.p_v.setLabel('left', 'v')
        self.p_w.setLabel('left', 'w')
        self.p_p.setLabel('left', 'p')
        self.p_q.setLabel('left', 'q')
        self.p_r.setLabel('left', 'r')



        # create curves to update later
        self.c_pn_t = self.p_pn.plot(name='truth')
        self.c_pe_t = self.p_pe.plot()
        self.c_pd_t = self.p_pd.plot()
        self.c_phi_t = self.p_phi.plot()
        self.c_theta_t = self.p_theta.plot()
        self.c_psi_t = self.p_psi.plot()
        self.c_u_t = self.p_u.plot()
        self.c_v_t = self.p_v.plot()
        self.c_w_t = self.p_w.plot()
        self.c_p_t = self.p_p.plot()
        self.c_q_t = self.p_q.plot()
        self.c_r_t = self.p_r.plot()

        # self.c_pn_e = self.p_pn.plot(name='estimate')
        # self.c_pe_e = self.p_pe.plot()
        # self.c_pd_e = self.p_pd.plot()
        # self.c_phi_e = self.p_phi.plot()
        # self.c_theta_e = self.p_theta.plot()
        # self.c_psi_e = self.p_psi.plot()
        # self.c_u_e = self.p_u.plot()
        # self.c_v_e = self.p_v.plot()
        # self.c_w_e = self.p_w.plot()
        # self.c_p_e = self.p_p.plot()
        # self.c_q_e = self.p_q.plot()
        # self.c_r_e = self.p_r.plot()

        self.c_pn_d = self.p_pn.plot(name='desired')
        self.c_pe_d = self.p_pe.plot()
        self.c_pd_d = self.p_pd.plot()
        self.c_phi_d = self.p_phi.plot()
        self.c_theta_d = self.p_theta.plot()
        self.c_psi_d = self.p_psi.plot()
        self.c_u_d = self.p_u.plot()
        self.c_v_d = self.p_v.plot()
        self.c_w_d = self.p_w.plot()
        self.c_p_d = self.p_p.plot()
        self.c_q_d = self.p_q.plot()
        self.c_r_d = self.p_r.plot()



        # initialize state variables
        self.time_t = 0
        self.pn_t = 0
        self.pe_t = 0
        self.pd_t = 0
        self.phi_t = 0
        self.theta_t = 0
        self.psi_t = 0
        self.u_t = 0
        self.v_t = 0
        self.w_t = 0
        self.p_t = 0
        self.q_t = 0
        self.r_t = 0

        # self.time_e = 0
        # self.pn_e = 0
        # self.pe_e = 0
        # self.pd_e = 0
        # self.phi_e = 0
        # self.theta_e = 0
        # self.psi_e = 0
        # self.u_e = 0
        # self.v_e = 0
        # self.w_e = 0
        # self.p_e = 0
        # self.q_e = 0
        # self.r_e = 0

        self.time_d = 0
        self.pn_d = 0
        self.pe_d = 0
        self.pd_d = 0
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0
        self.u_d = 0
        self.v_d = 0
        self.w_d = 0
        self.p_d = 0
        self.q_d = 0
        self.r_d = 0

        self.state_mat = {'pn_c':[],'pe_c':[],'pd_c':[],
                          'u_c':[],'v_c':[],'w_c':[],
                          'yaw_c':[],'pitch_c':[],'roll_c':[],
                          'p_c':[],'q_c':[],'r_c':[],
                          'time':[],
                          'pn':[],'pe':[],'pd':[],
                          'u':[],'v':[],'w':[],
                          'roll':[],'pitch':[],'yaw':[],
                          'p':[],'q':[],'r':[]}
        # truth/estimate storage lists
        # self.estimates = []
        self.truths = []
        self.desired = []

        # plot list
        self.p_list = [self.p_pn, self.p_pe, self.p_pd, self.p_phi, self.p_theta, self.p_psi, self.p_u, self.p_v, self.p_w, self.p_p, self.p_q, self.p_r]

        # curve lists
        self.c_list_t = [self.c_pn_t, self.c_pe_t, self.c_pd_t, self.c_phi_t, self.c_theta_t, self.c_psi_t, self.c_u_t, self.c_v_t, self.c_w_t, self.c_p_t, self.c_q_t, self.c_r_t]
        # self.c_list_e = [self.c_pn_e, self.c_pe_e, self.c_pd_e, self.c_phi_e, self.c_theta_e, self.c_psi_e, self.c_u_e, self.c_v_e, self.c_w_e, self.c_p_e, self.c_q_e, self.c_r_e]
        self.c_list_d = [self.c_pn_d, self.c_pe_d, self.c_pd_d, self.c_phi_d, self.c_theta_d, self.c_psi_d, self.c_u_d, self.c_v_d, self.c_w_d, self.c_p_d, self.c_q_d, self.c_r_d]


    # method for updating each states
    def update(self):
        # pack stored data into lists
        self.truths.append([self.time_t, self.pn_t, self.pe_t, self.pd_t, self.phi_t, self.theta_t, self.psi_t, self.u_t, self.v_t, self.w_t, self.p_t, self.q_t, self.r_t])
        # self.estimates.append([self.time_e, self.pn_e, self.pe_e, self.pd_e, self.phi_e, self.theta_e, self.psi_e, self.u_e, self.v_e, self.w_e, self.p_e, self.q_e, self.r_e])
        self.desired.append([self.time_d, self.pn_d, self.pe_d, self.pd_d, self.phi_d, self.theta_d, self.psi_d, self.u_d, self.v_d, self.w_d, self.p_d, self.q_d, self.r_d])
        # discard data outside desired plot time window
        for i in range(0,1000):
            if self.truths[0][0] < self.truths[-1][0] - self.t_win:
                self.truths.pop(0)

            # if self.estimates[0][0] < self.estimates[-1][0] - self.t_win:
            #     self.estimates.pop(0)

            if self.desired[0][0] < self.desired[-1][0] - self.t_win:
                self.desired.pop(0)

        # set the window widths
        for i in range(0,len(self.p_list)):
            # self.p_list[i].setLimits(xMin=self.truths[-1][0], xMax=self.truths[-1][0] + self.t_win)
            self.p_list[i].setLimits(xMin=self.truths[-1][0] - self.t_win, xMax=self.truths[-1][0])


        # stack the data lists
        truths_array = np.vstack(self.truths)
        time_t_array = truths_array[:,0]

        # estimates_array = np.vstack(self.estimates)
        # time_e_array = estimates_array[:,0]

        desired_array = np.vstack(self.desired)
        time_d_array = desired_array[:,0]

        # set the truth states
        for i in range(0,len(self.c_list_t)):
            self.c_list_t[i].setData(time_t_array, truths_array[:,i+1], pen=(255,0,0))

        # # set the estimated states
        # for i in range(0,len(self.c_list_e)):
        #     self.c_list_e[i].setData(time_e_array, estimates_array[:,i+1], pen=(0,255,0))

        # set the desired states
        for i in range(0,len(self.c_list_d)):
            self.c_list_d[i].setData(time_d_array, desired_array[:,i+1], pen=(0,255,0))
        # update the plotted data
        self.app.processEvents()
        self.update_state_mat()
        # sio.savemat('diff_flat_data.mat',self.state_mat)
        # sio.savemat(outfile,self.state_mat)
    def savecallback(self,event):
        location = self.folder + '/scripts/data/' + self.file
        sio.savemat(location,self.state_mat)
    def update_state_mat(self):
        self.state_mat['pn_c'].append(self.pn_d)
        self.state_mat['pe_c'].append(self.pe_d)
        self.state_mat['pd_c'].append(self.pd_d)
        self.state_mat['u_c'].append(self.u_d)
        self.state_mat['v_c'].append(self.v_d)
        self.state_mat['w_c'].append(self.w_d)
        self.state_mat['yaw_c'].append(self.psi_d)
        self.state_mat['pitch_c'].append(self.phi_d)
        self.state_mat['roll_c'].append(self.theta_d)
        self.state_mat['p_c'].append(self.p_d)
        self.state_mat['q_c'].append(self.q_d)
        self.state_mat['r_c'].append(self.r_d)
        self.state_mat['time'].append(self.time_t)

        self.state_mat['pn'].append(self.pn_t)
        self.state_mat['pe'].append(self.pe_t)
        self.state_mat['pd'].append(self.pd_t)
        self.state_mat['u'].append(self.u_t)
        self.state_mat['v'].append(self.v_t)
        self.state_mat['w'].append(self.w_t)
        self.state_mat['yaw'].append(self.psi_t)
        self.state_mat['pitch'].append(self.phi_t)
        self.state_mat['roll'].append(self.theta_t)
        self.state_mat['p'].append(self.p_t)
        self.state_mat['q'].append(self.q_t)
        self.state_mat['r'].append(self.r_t)

    def truthCallback(self, msg):
        # unpack positions and linear velocities
        self.pn_t = msg.pose.pose.position.x
        self.pe_t = msg.pose.pose.position.y
        self.pd_t = msg.pose.pose.position.z
        self.u_t = msg.twist.twist.linear.x
        self.v_t = msg.twist.twist.linear.y
        self.w_t= msg.twist.twist.linear.z

        # orientation in quaternion form
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_t = euler[0]
        self.theta_t = euler[1]
        self.psi_t = euler[2]
        self.p_t = msg.twist.twist.angular.x
        self.q_t = msg.twist.twist.angular.y
        self.r_t = msg.twist.twist.angular.z

        # unpack time
        if self.init_time == True:
            self.time0 = msg.header.stamp.to_sec()
            self.init_time = False
        self.time_t = msg.header.stamp.to_sec() - self.time0


    def estimateCallback(self, msg):
        pass
        # unpack positions and linear velocities
        # self.pn_e = msg.pose.pose.position.x
        # self.pe_e = msg.pose.pose.position.y
        # self.pd_e = msg.pose.pose.position.z
        # self.u_e = msg.twist.twist.linear.x
        # self.v_e = msg.twist.twist.linear.y
        # self.w_e = msg.twist.twist.linear.z

        # # orientation in quaternion form
        # quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w)
        #
        # # Use ROS tf to convert to Euler angles from quaternion
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        #
        # # unpack angles and angular velocities
        # self.phi_e = euler[0]
        # self.theta_e = euler[1]
        # self.psi_e = euler[2]
        # self.p_e = msg.twist.twist.angular.x
        # self.q_e = msg.twist.twist.angular.y
        # self.r_e = msg.twist.twist.angular.z
        #
        # # unpack time
        # if self.init_time == True:
        #     self.time0 = msg.header.stamp.to_sec()
        #     self.init_time = False
        # self.time_e = msg.header.stamp.to_sec() - self.time0

    def trajecoryCallback(self,msg):
        self.pn_d = msg.x
        self.pe_d = msg.y
        self.pd_d = msg.F#-1
        self.psi_d = msg.z
        # print self.pn_d

        # unpack time
        # if self.init_time == True:
        #     self.time0 = msg.header.stamp.to_sec()
        #     self.init_time = False
        self.time_d = self.time_t

    def xdesCallback(self,msg):
        self.u_d = msg.x
        self.v_d = msg.y
        self.w_d = msg.F
        # print self.pn_d

        # unpack time
        # if self.init_time == True:
        #     self.time0 = msg.header.stamp.to_sec()
        #     self.init_time = False
        self.time_d = self.time_t

    def uffCallback(self,msg):
        pass
        # self.phi_d = msg.x
    #     self.theta_d = msg.y
    #     self.u_d = msg.F
    #     self.r_d = msg.z
    #     # print self.pn_d
    #
    #     # unpack time
    #     # if self.init_time == True:
    #     #     self.time0 = msg.header.stamp.to_sec()
    #     #     self.init_time = False
    #     self.time_d = self.time_t

    def cmdCallback(self,msg):
        self.phi_d = msg.x
        self.theta_d = msg.y
        # self.u_d = msg.F
        self.r_d = msg.z
        # print self.pn_d

        # unpack time
        # if self.init_time == True:
        #     self.time0 = msg.header.stamp.to_sec()
        #     self.init_time = False
        self.time_d = self.time_t
################################################################################
################################################################################
################################################################################


def main():
    # initialize node
    rospy.init_node('state_plotter', anonymous=True)

    # initialize plotter class
    plotter = Plotter()

    # listen for messages and plot
    r = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        try:
            # plot the local positions of each vehicle
            plotter.update()

            # let it rest a bit

            r.sleep()

        except rospy.ROSInterruptException:
            # sio.savemat(diff_flat_data.mat,plotter.state_mat)
            print "exiting...."
            return

if __name__ == '__main__':
    main()
