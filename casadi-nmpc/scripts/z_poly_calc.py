#!/usr/bin/env python3

"""
z_poly_calc.py

Python ancillary controller calculation. Used as a ROS service to provide computational
geometry calculations for Tube MPC.

Keenan Albee, 2021
Libraries and assistance: Tor Heirung, Charles Oestreich, others.
"""

# Import required Python code.
# import roslib
import rospy
import numpy as np
import scipy
import scipy.signal
from pytope import Polytope
import matplotlib.pyplot as plt
import time

from trace_msgs.srv import TDMRPI_srv, TDMRPI_srvResponse
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

# also requires: pycddlib, libgmp-dev (on the repos) for pytope

# State and input convetion:
# x = [x1; x2; x3; x1d; x2d; x3d]
# u = [u1; u2; u3]

class ZPoly:
    def __init__(self):
        self.DT = 0.2  # delta-t, [s]
        self.m = 9.583  # mass, [kg]

        self.epsilon = 0.005  # mRPI calculation parameter

        # K_dr LQR settings
        self.Q1 = 5.0
        self.Q2 = 5.0
        self.Q3 = 5.0
        self.Q4 = 10.0
        self.Q5 = 10.0
        self.Q6 = 10.0
        self.R1 = 1000.0
        self.R2 = 1000.0
        self.R3 = 1000.0

        # Q = (5.0, 10.0), R = 1000.0 tends to work well for minimal mRPI
        self.Q = np.diag([self.Q1, self.Q2, self.Q3, self.Q4, self.Q5, self.Q6])
        self.R = np.diag([self.R1, self.R2, self.R3])
        self.Q_LQR = np.diag([5, 5, 5, 50, 50, 50])
        self.R_LQR = np.diag([0.1, 0.1, 0.1])

        ### 3D satellite translation, CONTINUOUS system
        A = np.vstack(
            (np.array([[0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 1]]),
                       np.zeros((3,6)))
            )

        B = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [1/self.m, 0,   0],
                      [0,   1/self.m, 0],
                      [0,   0,   1/self.m]])

        # convert continuous sytem to discrete using ZOH
        [self.A, self.B] = self.get_discrete_dynamics(A, B, self.DT)

        self.u_max = 0.4  # input, [N]
        self.U_max = np.array([[self.u_max, self.u_max, self.u_max]]).T

        self.W = np.array([[0.05, 0.05, 0.05, 0.01, 0.01, 0.01]]).T

    def initialize(self):
        """ Start up ROS spin.
        """
        rospy.init_node('z_poly', anonymous = True)
        s = rospy.Service('mrpi', TDMRPI_srv, self.handle_mrpi)
        print('mrpi service ready...')

        rospy.spin()
        return

    def check_LQR(self):
        """Check 3D case dlqr gain
        """
        ## Determine K_dr (use dLQR)
        K_dr, _, _ = self.dlqr(self.A, self.B, self.Q_LQR, self.R_LQR)
        K_dr = -K_dr  # correct definition for our use
        print(K_dr)

    def unit_test(self):
        """ Verify performance on a double integrator
        """
        print('Starting 1D double integrator unit test...')

        ### Set 1D double integrator, CONTINUOUS system
        self.A = np.array(
            [[0, 1],
            [0, 0]])

        m = 1.0
        self.B = np.array([[0, 1/m]]).T

        self.DT = 0.2

        [self.A, self.B] = self.get_discrete_dynamics(self.A, self.B, self.DT)

        self.u_max = 0.3  # input, [N]
        self.U_max = np.array([[self.u_max]]).T

        self.W = np.array([[0.01, 0.05]]).T

        self.Q_LQR = np.diag([10, 10])
        self.R_LQR = np.diag([10])

        # mRPI settings
        self.epsilon = 0.005

        [K_dr, Autight, butight, Z_poly, debug] = self.z_poly_calc(self.A, self.B, self.Q_LQR, self.R_LQR, self.W, self.U_max, self.DT)
        print(debug['A_cl'])
        print(debug['U_poly'].A)
        print(debug['U_poly'].b)
        print(debug['W_poly'].A)
        print(debug['W_poly'].b)
        print(Autight)
        print(butight)

        fig1, ax1 = plt.subplots(num=1)
        plt.grid()
        Z_poly.plot(ax1, facecolor='b')
        plt.axis([-0.8, 0.8, -0.4, 0.4])

        print('\nStarting Limon double integrator unit test...')
        """ See [1] D. Limon, I. Alvarado, T. Alamo, and E. Camacho,
        “On the design of robust tube-based MPC for tracking,” IFAC Proc. Vol., vol. 17, no. 1 PART 1, pp. 15333–15338, 2008.
        """

        ### Set 1D double integrator, multiple input, CONTINUOUS system
        self.A = np.array(
            [[0, 1],
            [0, 0]])

        self.B = np.array(
            [[-0.5, 0.25],
            [1.0, 0.5]])

        self.DT = 1.0

        [self.A, self.B] = self.get_discrete_dynamics(self.A, self.B, self.DT)

        self.u_max = 0.3  # input, [N]
        self.U_max = np.array([[self.u_max, self.u_max]]).T

        self.W = np.array([[0.1, 0.1]]).T

        self.Q_LQR = np.diag([1.0, 1.0])
        self.R_LQR = np.diag([10.0, 10.0])

        # mRPI settings
        self.epsilon = 0.005

        [K_dr, Autight, butight, Z_poly, debug] = self.z_poly_calc(self.A, self.B, self.Q_LQR, self.R_LQR, self.W, self.U_max, self.DT)
        print(debug['A_cl'])
        print(debug['U_poly'].A)
        print(debug['U_poly'].b)
        print(debug['W_poly'].A)
        print(debug['W_poly'].b)

        # fig1, ax1 = plt.subplots(num=1)
        # plt.grid()
        # Z_poly.plot(ax1, facecolor='b')
        # plt.axis([-0.8, 0.8, -0.4, 0.4])
        #
        # Utight_poly = Polytope(Autight, butight)  # input polytope
        # fig2, ax2 = plt.subplots(num=2)
        # plt.grid()
        # Utight_poly.plot(ax2, facecolor='r')
        # plt.axis([-0.4, 0.4, -0.4, 0.4])
        # plt.show()

        print('\nStarting Buckner unit test...')
        ### Set 1D double integrator, Envisat, CONTINUOUS system
        self.A = np.array(
            [[0, 1],
            [0, 0]])

        self.B = np.array([[0, 1.0/1500]]).T

        self.DT = 0.5

        [self.A, self.B] = self.get_discrete_dynamics(self.A, self.B, self.DT)

        print(self.A)
        print(self.B)

        self.u_max = 65.0  # input, [N]
        self.U_max = np.array([[self.u_max]]).T

        self.W = np.array([[7.9, 0.2]]).T

        self.Q_LQR = np.diag([10000, 10000])
        self.R_LQR = np.diag([100])

        # mRPI settings
        self.epsilon = 0.005

        [K_dr, Autight, butight, Z_poly, debug] = self.z_poly_calc(self.A, self.B, self.Q_LQR, self.R_LQR, self.W, self.U_max, self.DT)
        print(K_dr)
        print(debug['A_cl'])
        print(debug['U_poly'].A)
        print(debug['U_poly'].b)
        print(debug['W_poly'].A)
        print(debug['W_poly'].b)

        # fig1, ax1 = plt.subplots(num=1)
        # plt.grid()
        # Z_poly.plot(ax1, facecolor='b')
        # plt.axis([-700.0, 700.0, -100.0, 100.0])

        plt.show()

    def z_poly_calc(self, A, B, Q_LQR, R_LQR, W, U_max, DT):
        """ Calculate the mRPI for LQR disturbance rejection gain
        Inputs:
            A - discrete linear dynamics state matrix
            B - discrete linear dynamics input matrix
            Q_LQR - discrete LQR weighting matrix
            R_LQR - discrete LQR weighting matrix
            W - np [n, 1] of box constraints
            U_max - np [m, 1] of box constraints
            DT - timestep to convert to discrete time

        Outputs:
            K_dr - disturbance rejection gain
            Utight - tightened u constraints
            Xtight - tightened x constraints
            Z - mRPI epsilon polytope approximation
        """
        n = np.shape(A)[1]
        m = np.shape(B)[1]

        # box constraints
        Aw = np.vstack(
        (np.eye(n),
        -np.eye(n)))

        bw = np.vstack(
        (W,
        W))

        # actuation constraints
        Au = np.vstack(
        (np.eye(m),
        -np.eye(m)))

        bu = np.vstack(
        (U_max,
        U_max))

        # create polytopes using half space representation:
        # Polytope(A, b), A*x <= b and H = [A b]
        W_poly = Polytope(Aw, bw)  # disturbance polytope
        U_poly = Polytope(Au, bu)  # input polytope

        ## Determine K_dr (use dLQR)
        K_dr, _, _ = self.dlqr(A, B, Q_LQR, R_LQR)
        K_dr = -K_dr  # correct definition for our use

        A_cl = A + B@K_dr  # closed loop error dynamics evolution
        A_cl = np.squeeze(np.asarray(A_cl))  # np matrix --> array

        ## mRPI calculation
        Z_poly, _ = self.eps_MRPI(A_cl, W_poly, self.epsilon, 3)  # this is the mRPI

        U = np.zeros((3, 1))

        KZ_poly = K_dr*Z_poly  # the set K*Z
        Utight_poly = U_poly - KZ_poly  # the tightened U set
        Autight = Utight_poly.A
        butight = Utight_poly.b

        # print(W_poly.A)
        # print(W_poly.b)
        # print(KZ_poly.A)
        # print(bu)
        # print(Utight_poly.b)

        #
        # fig2, ax2 = plt.subplots()
        # plt.grid()
        # KZ_poly.plot(ax2, facecolor='b')
        # plt.axis([-0.8, 0.8, -0.4, 0.4])
        # plt.draw()
        # plt.title('KZ_poly')
        # plt.pause(0.001)
        #
        # fig3, ax3 = plt.subplots()
        # plt.grid()
        # Utight_poly.plot(ax3, facecolor='b')
        # plt.axis([-0.8, 0.8, -0.4, 0.4])
        # plt.draw()
        # plt.title('Utight_poly')
        # plt.pause(1000)

        debug = {'A_cl': A_cl, 'W_poly': W_poly, 'U_poly': U_poly}

        return [K_dr, Autight, butight, Z_poly, debug]

    def run_gains_test(self):
        """ Run test to determine best (feasible) LQR gains. Assumes 2D double integrator for now.
        """
        self.mass = 10.0
        A = np.array(
            [[0, 1],
            [0, 0]])

        B = np.array([[0, 1/self.mass]]).T

        W = np.array([[0.04, 0.002]]).T
        # W = np.array([[0.04, 0.008]]).T
        U_max = np.array([[0.4]]).T
        DT = 0.2

        [A, B] = self.get_discrete_dynamics(A, B, DT)

        Z_polys = []
        R_weights = [1E-5, 1E-4, 1E-3, 1E-2, 1E-1, 1.0, 1E1, 1E2, 1E3, 1E4, 1E5]
        # R_weights = [0.1]

        # Test for best double integrator gains
        print("Running gains test...")
        Q_LQR = np.diag([5.0, 50.0])
        for R in R_weights:
            R_LQR = np.diag([R])
            print("R is: ", R)
            [K_dr_i, Autight_i, butight_i, Z_poly, _] = self.z_poly_calc(A, B, Q_LQR, R_LQR, W, U_max, DT)  # K_dr is 1x2 Autight is 2x1, butight is 2x1
            Z_polys.append(Z_poly)
            print('inputs are:')
            print(Autight_i)
            print(butight_i)
            print(Z_poly.get_H_rep()[0].shape)
            # print(Z_poly.get_H_rep())

        # plot
        plt.ion()
        plt.show()
        dims = [-0.3, 0.3, -0.3, 0.3]

        for Z_poly in Z_polys:
            fig1, ax1 = plt.subplots()
            plt.grid()
            Z_poly.plot(ax1, facecolor='b')
            plt.axis(dims)
            plt.draw()
            plt.pause(0.001)
        plt.pause(1000)

    def get_discrete_dynamics(self, A, B, dt):
        """ Convert continuous sytem to discrete using ZOH
        """
        n = A.shape[0]
        m = B.shape[1]
        system = (A, B, np.eye(n), np.zeros((n, m)))
        system_d = scipy.signal.cont2discrete(system, dt)
        A = system_d[0]
        B = system_d[1]
        return [A, B]

    def handle_mrpi(self, req):
        """ Handle mrpi service request
        Inputs:
        std_msgs/Float64MultiArray w
        std_msgs/Float64MultiArray u_max
        float64 dt
        float64 mass
        float64 Q_pos_anc
        float64 Q_vel_anc
        float64 R_anc

        Outputs, which are all matrices to be used in the nominal MPC design:
        K_dr: [3, 6]
        Autight: [6, 3] (could change but unlikely)
        butight: [3, 1] (could change but unlikely)
        AZ: [unknown, 6]
        bZ: [unknown, 1]
        """
        w = req.w.data
        u_max = req.u_max.data
        dt = req.dt
        mass = req.mass
        Q_pos_LQR = req.Q_pos_anc
        Q_vel_LQR = req.Q_vel_anc
        R_LQR = req.R_anc

        Q_LQR = np.diag([Q_pos_LQR, Q_pos_LQR, Q_pos_LQR, Q_vel_LQR, Q_vel_LQR, Q_vel_LQR])
        R_LQR = np.diag([R_LQR, R_LQR, R_LQR])

        u_max = np.asarray([u_max]).T
        w_bound = np.asarray([w]).T

        [K_dr, Autight, butight, AZ_actual, bZ_actual] = self.calc_MRPI_and_K_dr(w_bound, dt, u_max, mass, Q_LQR, R_LQR)  # need to send out AZ and bZ on service

        # always convert to 100x6, 100x1 for CasADi
        AZ = np.zeros((100, 6))
        bZ = np.zeros((100, 1))
        AZ[0:AZ_actual.shape[0], 0:6] = AZ_actual
        bZ[0:AZ_actual.shape[0], 0:1] = bZ_actual

        K_dr_msg = Float64MultiArray()
        K_dr_msg.data = np.ndarray.flatten(K_dr).tolist()  # flattens row-major by default
        K_dr_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        K_dr_msg.layout.dim[0].size = np.shape(K_dr)[0]
        K_dr_msg.layout.dim[1].size = np.shape(K_dr)[1]

        Autight_msg = Float64MultiArray()
        Autight_msg.data = np.ndarray.flatten(Autight).tolist()
        Autight_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        Autight_msg.layout.dim[0].size = np.shape(Autight)[0]
        Autight_msg.layout.dim[1].size = np.shape(Autight)[1]

        butight_msg = Float64MultiArray()
        butight_msg.data = np.ndarray.flatten(butight).tolist()
        butight_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        butight_msg.layout.dim[0].size = np.shape(butight)[0]
        butight_msg.layout.dim[1].size = np.shape(butight)[1]

        AZ_msg = Float64MultiArray()
        AZ_msg.data = np.ndarray.flatten(AZ).tolist()
        AZ_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        AZ_msg.layout.dim[0].size = np.shape(AZ)[0]
        AZ_msg.layout.dim[1].size = np.shape(AZ)[1]

        bZ_msg = Float64MultiArray()
        bZ_msg.data = np.ndarray.flatten(bZ).tolist()
        bZ_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        bZ_msg.layout.dim[0].size = np.shape(bZ)[0]
        bZ_msg.layout.dim[1].size = np.shape(bZ)[1]

        print("...mrpi response sent.")

        # format is Au * u <= bu, for u = [ux, uy, uz]
        # need to bundle up outputs as MRPI service response
        # return MRPIResponse(K_dr, Autight, butight, AZ, bZ)
        return TDMRPI_srvResponse(K_dr_msg, Autight_msg, butight_msg, AZ_msg, bZ_msg)

    def calc_MRPI_and_K_dr(self, w_bound, dt, U_max, mass, Q_LQR, R_LQR):
        """ Set up the disturbance rejection controller and find the MRPI---shrink constraints.
        Uses a default epsilon setting for the mRPI approximation. Set up for the 3D double integrator dynamics.

        Inputs:
        w_bound - [6x1] of w disturbance bound
        dt - timestep
        U_max - u min/max restriction, [3x1]
        mass - system mass [kg]
        Q_LQR - to design ancillary gain
        R_LQR - to design ancillary gain

        Outputs, which are all matrices to be used in the nominal MPC design:
        K_dr: [3, 6]
        Autight: [6, 3] (could change but unlikely)
        butight: [3, 1] (could change but unlikely)
        AZ: [unknown, 6]
        bZ: [unknown, 1]
        """
        wx = w_bound[0,0]
        wy = w_bound[1,0]
        wz = w_bound[2,0]
        wdx = w_bound[3,0]
        wdy = w_bound[4,0]
        wdz = w_bound[5,0]
        dt = dt
        U_max = U_max     # input maxes, [N]
        self.m = mass  # base mass, [kg]
        m = mass
        self.Q_LQR = Q_LQR
        self.R_LQR = R_LQR

        W_all = np.array([[wx, wy, wz, wdx, wdy, wdz]]).T
        DT = dt
        Autight = np.zeros((6, 3))
        butight = np.zeros((6, 1))
        K_dr = np.zeros((3, 6))

        # set up normal double integrator
        A = np.array(
            [[0, 1],
             [0, 0]])

        B = np.array([[0, 1/m]]).T

        # convert continuous sytem to discrete using ZOH
        [A, B] = self.get_discrete_dynamics(A, B, DT)

        # Run on 3 different double integrators for different W
        Z_polys = []
        for i in range(3):
            W = np.array([[ W_all[i, 0], W_all[i+3, 0] ]]).T
            Q_LQR = np.diag([self.Q_LQR[i, i], self.Q_LQR[i+3, i+3]])
            R_LQR = np.diag([self.R_LQR[i, i]])

            [K_dr_i, Autight_i, butight_i, Z_poly, _] = self.z_poly_calc(A, B, Q_LQR, R_LQR, W, U_max[i,0], DT)  # K_dr is 1x2 Autight is 2x1, butight is 2x1
            if butight_i[0] < 0.0:
                print(butight_i)
                print('Not enough input to handle uncertainty! Defaulting to secondary minimums for w_bound...')
                W[1] = W[1]/2  # ease up on velocity uncertainties
                # Q_LQR = np.diag([5, 10])
                # R_LQR = np.diag([2.0])
                [K_dr_i, Autight_i, butight_i, Z_poly, _] = self.z_poly_calc(A, B, Q_LQR, R_LQR, W, U_max[i,0], DT)  # K_dr is 1x2 Autight is 2x1, butight is 2x1

                if butight_i[0] < 0.0:
                    print('Using default input shrinkage...')
                    butight_i = np.array([[0.2, 0.2]]).T  # if all else fails...

            # assemble matrix for combined system
            K_dr[[i], i] = K_dr_i[0, 0]
            K_dr[[i], i+3] = K_dr_i[0, 1]
            Autight[i*2:i*2 + 2, [i]] = Autight_i
            butight[i*2:i*2 + 2, [0]] = butight_i[1]
            Z_polys.append(Z_poly)

        # reassemble Z to 6DOF form
        AZ = np.empty((0,6))
        bZ = np.empty((0,1))
        for i in range(len(Z_polys)):
            Z = Z_polys[i]

            nrows = np.shape(Z.A)[0]
            AZ_next = np.zeros((nrows, 6))
            bZ_next = np.zeros((nrows, 1))

            AZ_next[:, [i]] = Z.A[:, [0]]
            AZ_next[:, [i+3]] = Z.A[:, [1]]
            bZ_next[:, [0]] = Z.b

            AZ = np.vstack(
            (AZ,
            AZ_next))

            bZ = np.vstack(
            (bZ,
            bZ_next))

        # plot
        # plt.ion()
        # plt.show()
        # dims = [-3.0, 3.0, -3.0, 3.0]
        #
        # for Z_poly in Z_polys:
        #     fig1, ax1 = plt.subplots()
        #     plt.grid()
        #     Z_poly.plot(ax1, facecolor='b')
        #     plt.axis(dims)
        #     plt.draw()
        #     plt.pause(0.001)
        # plt.pause(1000)

        return K_dr, Autight, butight, AZ, bZ

    def dlqr(self, A, B, Q, R):
        """ Solve the discrete time lqr controller. Credit to Mark Mueller for wrapper.

        x[k+1] = A x[k] + B u[k]

        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """
        #ref Bertsekas, p.151

        #first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

        #compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))

        eigVals, eigVecs = scipy.linalg.eig(A-B*K)

        return K, X, eigVals

    def eps_MRPI(self, A, W, epsilon, s_max=20):
        """ Credit: Tor Heirung, UC Berkeley

        Determines an outer epsilon-approximation of a minimal RPI set.

        Implements Algorithm 1 Raković et al. [1] for determining an outer
        epsilon-approximation of the minimal RPI set for the autonomous system
        x+ = A*x + w
        using the following algorithm (copied directly from the paper and referenced
        throughout the code):

        ALGORITHM 1: Computation of an RPI, outer epsilon-approximation of the MRPI
        set F_infinity
        REQUIRE: A, W, and epsilon > 0
        ENSURE: F(alpha, s) such that
        F_infinity <= F(alpha, s) <= F_infinity + B_infinity^n(epsilon)
        1: Choose any s in N (ideally, set s <- 0).
        2: REPEAT
        3:   Increment s by one.
        4:   Compute alpha^o(s) as in (11) and set alpha <- alpha^o(s).
        5:   Compute M(s) as in (13).
        6: UNTIL alpha <= epsilon / (epsilon + M(s))
        7: Compute F_s as the Minkowski sum (2) and scale it to give
         F(alpha, s) := (1 - alpha)^(-1) * F_s.

        The s-term Minkowski sum (2) is computed in V-rep; computing the sum in
        H-rep can be both slow and numerically more challenging.

        Args:
        A: A numpy array (the state transition matrix --  must be strictly stable).
        W: A Polytope instance that bounds the disturbance w (must be compact and
        contain the origin).
        epsilon: A (positive) error bound (the radius of the infinity-norm ball).
        s_max: An optional maximum value of s, at which the algorithm terminates.

        Returns:
        F_alpha_s: A Polytope instance that is the outer-epsilon approximation of
        the MRPI set for (A, W).
        result: A dict with keys
        alpha: A scalar in [0, 1]:  A^s W subset alpha W  (Eq. (4)).
        s: A positive integer:  F_alpha_s := (1 - alpha)^(-1) F_s  (Eq. (5)).
        M: A numpy array (shape (s + 1,)) of the numbers M(k), k = 0, ..., s.
        The last element, M[-1], is M(s), which satisfies
        alpha <= epsilon / (epsilon + M(s))  (Eq. (14)).
        status: 0 if the algorithm terminated successfully, otherwise -1.
        alpha_o_s: A numpy array (shape (s + 1,)) of the number alpha at
        every iteration k = 0, ..., s.
        F_s: A numpy array of Polytope instances: F_s[s] is the s-term Minkowski
        sum from i = 0 to s over A^i W  (Eq. (2)).
        eps_min: The minimal epsilon that does not require increasing s.

        Raises:
        ValueError: An argument did not satisfy a necessary condition or the support
        function could not be evaluated successfully.

        Paper reference:
        [1] Raković, S.V., Kerrigan, E.C., Kouramas, K.I., & Mayne, D.Q. (2005).
        Invariant approximations of the minimal robust positively invariant set. IEEE
        Transactions on Automatic Control, 50(3), 406-410.
        """
        status = -1  # set to 0 at successful termination (as in SciPy's linprog)

        m, n = A.shape
        if m != n:
            raise ValueError('A must be a square matrix')

        # The disturbance set W is in the form
        # W := {w in R^n | f_i' * w <= g_i, i in I}
        W.minimize_V_rep()
        F = W.A  # the columns f_i of A in the H-rep [A b] of W
        g = W.b  # the right-hand side b in the H-rep [A b] of W
        I = g.size  # the number of inequalities in the H-rep of W

        if not all(g > 0):
            raise ValueError('W does not contain the origin: g > 0 is not satisfied')

        # array of upper bounds on alpha values -- the scaling factor in the subset
        # condition A^s W subset alpha * W  (Eq. (10))
        alpha_o_s = np.full(s_max, np.nan)

        # To determine M(s) (used to bound the approximation error on F(alpha, s)):
        # Store support functions for each power of A, A^(s-1),
        # and each direction j = 1, ..., n. One row per s, each row has n support
        # functions for A^(s-1) positive and n for A^(s-1) negative; see (13).
        # M(s) is the maximum of all elements of each row s.
        # Store all values used to determine M(s) -- this is not necessary but useful
        # when debugging numerically challenging cases. Note that the first row
        # (s = 0) remains all zero (which is OK).
        M_s_row = np.zeros((s_max, 2 * n))  # each M(s) is the max over 2n values
        M = np.full(s_max, np.nan)  # M[s] is that maximum for each s

        # Pre-compute all powers of A, A^s, s = 0, ..., s_max
        A_pwr = np.stack([np.linalg.matrix_power(A, i) for i in range(s_max)])

        alpha_o = np.full(I, np.nan)

        # Step 1: Choose any s in N [natural numbers] (ideally, set s <- 0).
        s = 0

        # Step 2: repeat
        while s < s_max - 1:

            # Step 3: Increment s by one.
            s += 1

            # Step 4: Compute alpha^o(s) as in (11) and set alpha <- alpha^o(s).
            # alpha^o(s) = max_{i in I) h_W((A^s)' f_i) / g_i
            for i in range(I):
              fi = F[i, :].T
              h_W_i, status = W.support(A_pwr[s].T @ fi)
              if not status.success:
                print('Unsuccessful evaluation of the support function '
                      'h_W((A^{s})'' * f_{s}): {status.message}')
              alpha_o[i] = h_W_i / g[i]
            alpha_o_s[s] = np.max(alpha_o)
            alpha = alpha_o_s[s]

            # Step 5: Compute M(s) as in (13).
            # M(s) = max_j {sum_i(h_W_sm1_pos_j), sum_i(h_W_sm1_neg_j)}  (Eq. (13))
            # At iteration s, evaluate the support for the rows of A^(s-1) and use the
            # supports evaluated at previous iterations s to evaluate the sum over i,
            # i = 0, ..., s - 1.
            h_W_sm1_pos_j = np.full(n, np.nan)  # h_W((A^(s-1))' * e_j, j = 0, ..., n-1
            h_W_sm1_neg_j = np.full(n, np.nan)  # h_W((-A^(s-1))' * e_j, j = 0, ..., n-1
            # Evaluate support in direction +- (A^i)' * e_j, with e_j the jth standard
            # basis vector in R^n. That is, (A^i)' * e_j is the jth column of (A^i)', or
            # the jth row of A^i (A_pwr_i[j])
            for j in range(n):
              A_pwr_i = A_pwr[s - 1]  # i = 0, ..., s - 1

              h_W_sm1_pos_j[j], status_lhs = W.support(A_pwr_i[j])  # basis vector and transpose skipped here for simplicity
              h_W_sm1_neg_j[j], status_rhs = W.support(-A_pwr_i[j])
              if not all(status.success for status in (status_lhs, status_rhs)):
                raise ValueError('Unsuccessful evaluation of the support function in '
                                 'the direction of row {j} of A^{s - 1} (s = {s})')
            # Store all 2n support-function evaluations for this iteration s. That is,
            # {h_W((A^(s-1))' * e_j,  h_W((-A^(s-1))' * e_j}, j = 0, ..., n-1:
            M_s_row[s] = M_s_row[s - 1] + np.concatenate((h_W_sm1_pos_j, h_W_sm1_neg_j))
            # Take the sum over i from 0 to s - 1 (so include row s, hence ": s + 1"
            # M_s_argument = np.sum(M_s_row[s], axis=0)
            M[s] = np.max(M_s_row[s])  # Eq. (13), see above

            # Step 6: until alpha <= epsilon / (epsilon + M(s))
            # print(alpha, epsilon, epsilon / (epsilon + M[s]))
            if alpha <= epsilon / (epsilon + M[s]):
              status = 0  # success
              print('Minkowski sum iteration bound (s) obtained: ', s)
              break

        s_final = s

        # Step 7: Compute F_s as the Minkowski sum (2) and scale it to give
        # F(alpha, s) = (1 - alpha)^(-1) F_s.
        # F_s = sum_{i = 0}^{s - 1} A^i W,  F_0 = {0}  (Eq. (2))
        # print('Beginning Minkowski sums...')
        F_s = np.full(s_final + 1, Polytope(n=n))  # F_s, s = 0, ..., s_final
        for s in range(1, s_final + 1):  # determine F_s for s = 1, ..., s_final
            s
            F_s[s] = F_s[s - 1] + (A_pwr[s - 1] * W)  # F_s[0] is empty
            F_s[s].minimize_V_rep()  # critical when s_final is large
            # Scale to obtain the epsilon-approximation of the minimal RPI:
        F_alpha_s = F_s[s_final] * (1 / (1 - alpha))
        # TODO: Improve performance for large s_final by not constructing polytopes
        # for every s -- instead compute the vertices directly for every power of A
        # and add them together at the end (and finally remove redundant vertices)

        # The smallest epsilon for s_final terms in the Minkowski sum:
        eps_min = M[s_final] * alpha / (1 - alpha)

        result = {'alpha': alpha, 's': s_final, 'M': M[: s_final + 1],
                'status': status, 'alpha_o_s': alpha_o_s[: s_final + 1], 'F_s': F_s,
                'eps_min': eps_min}

        return F_alpha_s, result

    def unit_test_td(self):
        """ Run using TD parameters.
        """
        w_bound = np.array([[0.0331, 0.0260, 0.0537, 0.0069, 0.0055, 0.0073]]).T
        dt = 0.2
        U_max = np.array([[0.4, 0.4, 0.4]]).T
        mass = 10.0
        Q_LQR = np.diag([5, 5, 5, 50, 50, 50])
        R_LQR = np.diag([0.1, 0.1, 0.1])

        [K_dr, Autight, butight, AZ, bZ] = z_poly.calc_MRPI_and_K_dr(w_bound, dt, U_max, mass, Q_LQR, R_LQR)
        print('K_dr\n', K_dr)
        print('Autight\n', Autight)
        print('butight\n', butight)
        print(AZ.shape)
        print(bZ.shape)

if __name__ == '__main__':
    # Initialize TubeMPC
    z_poly = ZPoly()
    # z_poly.unit_test_td()
    # z_poly.run_gains_test()

    # z_poly.unit_test()
    # z_poly.unit_test_3d()

    # Start ROS loop for mRPI service
    z_poly.initialize()
