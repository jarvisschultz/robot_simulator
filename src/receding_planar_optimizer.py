#!/usr/bin/env python
"""
Jarvis Schultz
September 2013

This program offers a service to receding_planar_controller.py.  In the service
handler, we request the desired state of the system at the next dt.  Then we
perform a full trajectory optimization and reply with the appropriate controls
to send to the system.

SUBSCRIPTIONS:

PUBLISHERS:
    - PlanarControlLaw (receding_controls)

SERVICES:
    - PlanarSystemService (get_receding_state) (client)
    - PlanarControlLawService (get_receding_controller) (provider)

"""

## ros imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
from puppeteer_msgs.srv import PlanarControlLawService
from puppeteer_msgs.msg import PlanarControlLaw
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import MultiArrayDimension

## trep and misc.
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
from math import sin, cos, floor
from math import pi as mpi
import numpy as np
import sys
import scipy.io as sio
import os
import copy
import time
import threading

## internal
import receding_planar_controller as rp

from matplotlib.pyplot import hold, plot
import matplotlib.pyplot as mp


## global constants:
DTopt = 1/30.
WINDOW = 1.0
OFFSET = 1

## PRIMARY CLASS WITH CALLBACKS ##############################
class RecedingOptimizer:
    """
    This class will define all subscribers and service handlers.
    """
    def __init__(self):
        rospy.loginfo("Starting receding-horizon optimizer...")
        # let's create a local instance of the MassSystem2D() that we can
        # perform our optimizations with:
        self.system = rp.MassSystem2D()

        # let's wait for and call the get_ref_config service to get the full
        # reference trajectory information from receding_planar_controller.py:
        rospy.loginfo("Waiting for reference service to be available...")
        rospy.wait_for_service("get_ref_config")
        get_ref_config = rospy.ServiceProxy("get_ref_config", PlanarSystemService)

        req = PlanarSystemServiceRequest(index=0)
        resp = get_ref_config(req)
        self.Xref = [resp.state]
        self.Uref = [resp.input]
        self.dt = resp.dt
        self.tref = [resp.time]
        self.length = resp.length
        # now we can fully fill in the reference trajectory
        for i in range(1, self.length-1):
            req = PlanarSystemServiceRequest(index=i)
            resp = get_ref_config(req)
            self.Xref.append(resp.state)
            self.Uref.append(resp.input)
            self.tref.append(resp.time)
        req = PlanarSystemServiceRequest(index=self.length-1)
        resp = get_ref_config(req)
        self.Xref.append(resp.state)
        self.tref.append(resp.time)
        self.Xref_original = copy.deepcopy(self.Xref)
        self.tref_original = copy.deepcopy(self.tref)
        self.Uref_original = copy.deepcopy(self.Uref)

        rospy.loginfo("Optimizer successfully received entire trajectory")

        # define all publishers:
        self.cont_pub = rospy.Publisher("receding_controls", PlanarControlLaw)
        # define all service providers:
        self.cont_serv = rospy.Service("get_receding_controller",
                                       PlanarControlLawService,
                                       self.control_service_handler)
        # define all subscribers:
        self.filt_sub = rospy.Subscriber("filt_state", PlanarSystemState,
                                         self.meascb)
        # define miscellaneous variables related to the receding horizon controller:
        self.system.create_dsys(np.array(self.tref))
        # perform linearization about current trajectory so that I can give it
        # to any service requesters:
        self.K, self.A, self.B = self.system.dsys.calc_feedback_controller(
            np.array(self.Xref), np.array(self.Uref), return_linearization=True)

        # initialize all variables that are needed for the receding horizon
        # optimization
        self.Xfilt = []
        self.first_flag = True
        self.base_time = rospy.Time.from_sec(0)
        self.Xfilt_len = 0
        if rospy.has_param("window_length"):
            self.window_length = rospy.get_param("window_length")
        else:
            self.window_length = WINDOW
            rospy.set_param("window_length", self.window_length)
        self.optimization_length = int(floor(self.window_length/self.dt))
        # manually open up another thread that repeatedly performs the full
        # discrete optimization as fast as possible:
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.optimizer)
        self.thread.start()
        return



    def optimizer(self):
        """
        This is a function that is given its own thread, and it will continually
        build a new reference trajectory and re-optimize.
        """
        rospy.loginfo("Started the optimizer function in %s"%
                      threading.currentThread().getName())
        while not rospy.is_shutdown():
            # first let's check if the length of the filtered vector is longer than
            # the last time we did the optimization
            with self.lock:
                Xfilt_local = copy.deepcopy(self.Xfilt)
                Xfilt_len_local = copy.deepcopy(self.Xfilt_len)
                Xref_local = copy.deepcopy(self.Xref)
                Uref_local = copy.deepcopy(self.Uref)
                tref_local = copy.deepcopy(self.tref)
                oplen_local = copy.deepcopy(self.optimization_length)
                A_local = copy.deepcopy(self.A)
                B_local = copy.deepcopy(self.B)
                K_local = copy.deepcopy(self.K)
            if len(Xfilt_local) > Xfilt_len_local+OFFSET:
                with self.lock:
                    self.Xfilt_len = len(Xfilt_local)
            else:
                rospy.logdebug("No new information for the optimization...")
                rospy.sleep(1/30.)
                continue
            kindex = len(Xfilt_local)-OFFSET
            with self.lock:
                self.system.create_dsys(tref_local[kindex:kindex+oplen_local])
            # if we got here, we can do a new optimization:
            Qcost = np.diag([100000, 100000, 0.1, 0.1, 0.1, 0.1, 3000, 3000])
            Rcost = np.diag([0.1, 0.1])
            def Qfunc(kf): return Qcost
            def Rfunc(kf): return Rcost
            # build trajectories:
            Xref = np.vstack((Xfilt_local[-OFFSET:],
                              Xref_local[kindex+OFFSET:kindex+oplen_local]))
            Uref = Xref[1:,2:4]
            # Xref = np.vstack((Xref_local[-OFFSET:],
            #                   Xref_local[kindex+OFFSET:kindex+oplen_local]))
            # Uref = Xref[1:,2:4]
            # print "offset = ",OFFSET,"kindex = ",kindex,"oplen = ",oplen_local,"len(Xref) = ",len(Xref)

            # build initial guesses:
            # X0 = np.array(copy.deepcopy(Xref_local)[kindex:kindex+oplen_local])
            # U0 = np.array(copy.deepcopy(Uref_local)[kindex:kindex+oplen_local-1])
            # (Xp,Up) = self.system.dsys.project(X0,U0)
            # Xp = copy.deepcopy(X0)
            # Up = copy.deepcopy(U0)
            X0 = np.array(copy.deepcopy(Xref))
            U0 = np.array(copy.deepcopy(Uref))
            self.system.sys.q = X0[0,0:4]
            self.system.sys.satisfy_constraints(keep_kinematic=True)
            Xp = np.zeros(X0.shape)
            Up = copy.deepcopy(U0)
            Xp[0] = X0[0]
            Xp[0,0:4] = self.system.sys.q
            self.system.dsys.set(Xp[0], Up[0], 0)
            for u in Up[1:]:
                Xp[self.system.dsys.k+1] = self.system.dsys.f()
                self.system.dsys.step(u)
            Xp[-1] = self.system.dsys.f()
            
            # zero kinematic velocity inputs
            Xref[:,6:8] *= 0
            cost = discopt.DCost(np.array(Xref), np.array(Uref), Qcost, Rcost)
            with self.lock:
                optimizer = discopt.DOptimizer(self.system.dsys, cost)
            optimizer.optimize_ic = False
            optimizer.descent_tolerance = 1e-8
            optimizer.first_method_iterations = 0
            optimizer.armijo_beta = 0.99
            finished = False
            # print np.array(tref_local[kindex:kindex+oplen_local]).shape
            # print Xref.shape, (np.array(Xref_local[kindex:kindex+oplen_local])).shape
            # print Uref.shape, (np.array(Uref_local[kindex:kindex+oplen_local-1])).shape

            try:
                finished, X, U = optimizer.optimize(Xp,
                                                    Up,
                                                    max_steps=90)
            except trep.ConvergenceError as e:
                rospy.logwarn("Detected optimization problem: %s",e.message)
            if not finished:
                rospy.logwarn("Optimization failed!!!")
                pass
            else:
                with self.lock:
                    pass
                    # then we can get the new controller:
                    K, A, B = self.system.dsys.calc_feedback_controller(X, U,
                                                Q=Qfunc, R=Rfunc, return_linearization=True)
                    Utmp = np.vstack((Uref_local[0:kindex], U, Uref_local[kindex+oplen_local-1:]))
                    Xtmp = np.vstack((Xref_local[0:kindex], X, Xref_local[kindex+oplen_local:]))
                    Atmp = np.vstack((A_local[0:kindex], A, A_local[kindex+oplen_local:]))
                    Btmp = np.vstack((B_local[0:kindex], B, B_local[kindex+oplen_local:]))
                    Ktmp = np.vstack((K_local[0:kindex], K, K_local[kindex+oplen_local-1:]))
                    self.Xref = Xtmp.copy()
                    self.Uref = Utmp.copy()
                    self.A = Atmp.copy()
                    self.B = Btmp.copy()
                    self.K = Ktmp.copy()

            if len(Xfilt_local) > 3 and finished:
                print "Writing out optimization, and breaking optimization thread"
                print "We have received a total of",len(Xfilt_local),"states"
                dat = {}
                with self.lock:
                    dat['Xoriginal'] = self.Xref_original
                    dat['Uoriginal'] = self.Uref_original
                    dat['tref'] = self.tref_original
                dat['Xcurrent'] = Xref
                dat['Ucurrent'] = Uref
                dat['Xfilt'] = Xfilt_local
                dat['X0'] = np.array(Xref_local[kindex:kindex+oplen_local])
                dat['U0'] = np.array(Uref_local[kindex:kindex+oplen_local-1])
                dat['kindex'] = kindex
                dat['offset'] = OFFSET
                dat['oplen'] = oplen_local
                # dat['Xopt'] = X
                # dat['Uopt'] = U
                fname = '/home/jarvis/Desktop/misc/debug_data/receding_debug/data.mat'
                sio.savemat(fname, dat, appendmat=False)
                hold(True)
                l1 = mp.plot(tref_local, np.array(self.Xref_original)[:,0:4],'k-',lw=2)
                l2 = mp.plot(tref_local[kindex:kindex+oplen_local], np.array(Xref)[:,0:4],'r-*',lw=2)
                l3 = mp.plot(tref_local[0:len(Xfilt_local)], np.array(Xfilt_local)[:,0:4],'b-',lw=2)
                l4 = mp.plot(tref_local[kindex:kindex+oplen_local], np.array(X)[:,0:4], 'g:',lw=2)
                l5 = mp.plot(tref_local, np.array(Xtmp)[:,0:4],'m--',lw=2)
                mp.legend((l1[0],l2[0],l3[0],l4[0],l5[0]), ("Original Reference",
                                                            "Optimization Reference",
                                                            "Filtered Data",
                                                            "Optimal Solution",
                                                            "Stitched Solution"),
                                                            loc='lower right')
                # l1 = mp.plot(tref_local[0:-1], np.array(self.Uref_original)[:,0:],'ko',lw=2)
                # l2 = mp.plot(tref_local[kindex:kindex+oplen_local-1], np.array(U)[:,0:],'r-*',lw=2)
                # l3 = mp.plot(tref_local[0:-1], np.array(Utmp)[:,0:],'mx',lw=2)
                # mp.legend((l1[0],l2[0],l3[0]), ("Original Input",
                #                                 "Optimal Solution",
                #                                 "Stitched Input"))
                # mp.xlim([tref_local[kindex-10], tref_local[kindex+oplen_local+10]])
                hold(False)
                mp.show()
                mp.cla()
                mp.clf()
                mp.close()
                break

        rospy.loginfo("Optimizer thread exiting")
        return


    # this callback is responsible for obtaining the filtered results from the
    # controller program, and adding them to the local trajectory of filtered
    # measurements.  We must use locks to remain thread safe with the
    # optimization thread.
    def meascb(self, data):
        rospy.logdebug("receding_optimizer entered its measurement callback")
        if self.first_flag:
            self.base_time = data.header.stamp
            self.first_flag = False

        Xtmp = [1]*self.system.dsys.nX
        Xtmp[0] = data.xm
        Xtmp[1] = data.ym
        Xtmp[2] = data.xr
        Xtmp[3] = data.r
        Xtmp[4] = data.pxm
        Xtmp[5] = data.pym
        Xtmp[6] = data.vxr
        Xtmp[7] = data.vr

        with self.lock:
            self.Xfilt.append(Xtmp)
        return


    # this is the service handler:
    def control_service_handler(self, req):
        """
        Return the feedforward and feedback data for the trajectory base on the
        current optimization.
        """
        con = PlanarControlLaw()
        if req.t != 0:
            # then use time, otherwise use the index
            try:
                index = [i for i,x in enumerate(self.tref) \
                         if x <= req.t < self.tref[i+1]][0]
            except:
                rospy.logerr("Invalid time for service request!")
                return None
            if index > len(self.tref)-1:
                rospy.logerr("Requested index is too large!")
                return None
        else:
            # use the index:
            if req.index > len(self.tref)-1:
                rospy.logerr("Requested index is too large!")
                return None
            index = req.index
        # print "index = ",index, "len(K) = ",len(self.K), "len(X) = ",len(self.Xref),"len(Uref) = ", len(self.Uref)
        kkey = np.ravel(self.K[index]).tolist()
        uffkey = self.Uref[index]
        con.header.frame_id = "/optimization_frame"
        con.header.stamp = rospy.Time.now()
        con.uff = uffkey
        con.K.layout.dim.append(MultiArrayDimension())
        con.K.layout.dim.append(MultiArrayDimension())
        con.K.layout.dim[0].label = "height"
        con.K.layout.dim[0].size = 2
        con.K.layout.dim[0].stride = 16
        con.K.layout.dim[1].label = "width"
        con.K.layout.dim[1].size = 8
        con.K.layout.dim[1].stride = 8
        con.K.data = kkey
        return con



def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('receding_optimizer', log_level=rospy.INFO)

    try:
        sim = RecedingOptimizer()
    except rospy.ROSInterruptException: pass
    rospy.spin()


if __name__=='__main__':
    main()
