#!/usr/bin/python
import math
import numpy as np
from tool_functions import *

def get_transformation_body(M, blist, thetalist):
    '''
    Returns the SE(2) transformation matrix using spatial screws and their angles.
    sample input: (using a 2R robot)
        blist: body screw axes of joint 1 thru joint n
            blist = np.array([[1, 0.2, 0.2],
                          [1, 2,   0,]])
        thetalist = np.array([0.2, 1.1])
    sample output: SE(2) transformation matrix of the body in s -  Tsb
    '''
    T = np.copy(M)
    for row in range(len(blist)):
        b = blist[row]
        theta = thetalist[row]
        T = T.dot(twist_2_SE2(b*theta))
    return T

def Jacobian_body(blist, thetalist):
    '''
    Returns the body jacobian for 2D applications.
    Example Input: (2R robot)
    blist = np.array([[1, 0.2, 0.2],        (each row is the screw axis of joint 1 thru joint n)
                      [1, 0,   3]])
    thetalist = np.array([0.2, 1.1])
    Example output:
    3x2 Body Jacobian
    '''
    Jb = np.zeros((3, len(blist)))
    T = np.eye(3)
    for i in reversed(range(len(blist))):
        b = blist[i]
        Jb[:,i] = get_adjoint(T).dot(b)
        #for the next iteration:
        theta =  thetalist[i]
        T_new = twist_2_SE2(-b*theta)
        T = T.dot(T_new)

    return Jb


def ik_body(M, Tsd, thetalist0, blist, e_theta = 0.01, e_v = 0.001):
    '''
    Calculates joint angles using body screw axes

    Example Input: (2R robot)
    M: SE2 matrix
        M = np.array([[1.0, 0, 0],[0, 1.0, 0], [0, 0, 1.0]])
    Tsd: desired end-effector transformation matrix in SE(2)
        Tsd = np.array([[1.0, 0, 0],[0, 1.0, 0], [0, 0, 1.0]])
    blist: each row is the screw axis of joint 1 thru joint n)
        blist = np.array([[1, 0.2, 0.2],
                          [1, 0,   3]])
    thetalist0: joint angles of the robot in its current position, which also serves as an initial guess for the Newton-Raphson process
        thetalist0 = np.array([0.2, 1.1])
    e_theta: small error tolerance for the end-effector linear orientation error. The returned joint angles must give an end-effector
    orientation error less than e_theta
        e_theta = 0.01
    e_v: small error tolerance on the end-effector linear position error. The returned joint angles must give an end-effector
    position error less than ev
        e_v = 0.001

    Example output:
    thetalist_new: joint angles that achieves the desired end-effector transformation
        thetalist_new = np.array([0.2, 1.1])
    '''
    Tsb = get_transformation_body(M, blist, thetalist0)
    thetalist_new = np.copy(thetalist0)
    maxiterations = 40
    i = 0
    success = False
    while True:
        Tbd = (np.linalg.inv(Tsb)).dot(Tsd)
        vb = SE2_2_twist(Tbd)
        if abs(vb[0])<=e_theta and np.linalg.norm(vb[1:]) <= e_v:
            success = True
            break
        i += 1
        if i > maxiterations:
            break
        Jb = Jacobian_body(blist, thetalist_new)
        Jb_T = np.linalg.pinv(Jb, rcond = 1e-16)
        thetalist_new = thetalist_new + Jb_T.dot(vb)
        Tsb = get_transformation_body(M, blist, thetalist_new)

    return thetalist_new, success

if __name__=='__main__':
    # slist = np.array([[1, 0, 0],
    #                   [1, 0, 0]])
    # M = np.array([[1.0, 0.0, 1.0],
    #               [0.0, 1.0, 0.0],
    #               [0.0, 0.0, 1.0]])
    # theta_list = np.array([np.pi/2, -np.pi/2])
    # print get_transformation_spatial(M,slist, theta_list)
    blist = np.array([[1.0, 0, 2.0],
                  [1.0, 0, 1.0]])
    thetalist = np.array([0.0, 0.0])
    blist_p = np.array([[0, 0, 1.0, 0, 2.0, 0],[0, 0, 1.0, 0, 1.0, 0]]).T
    M = np.array([[1.0, 0, 2.0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    M_p = np.array([[1.0, 0, 0, 2.0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 1.0]])

    e_theta = 0.01
    e_v = 0.001
    # Tsd = np.array([[0, 1.0, 0],
    #                 [-1, 0, -2.0],
    #                 [0, 0, 1.0]])       #(return [pi, 0])
    # Tsd_p = np.array([[0, 1.0, 0, 0],
    #                 [-1.0, 0, 0, -2.0],
    #                 [0, 0, 1.0, 0],
    #                 [0, 0, 0, 1.0]])       #(return [pi, 0])
    Tsd = np.array([[-1, 0, -2],
                    [0, -1, 0],
                    [0, 0, 1.0]])       #(return [pi, 0])
    Tsd_p = np.array([[-1, 0, 0, -2],
                    [0, -1, 0,  0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 1.0]])       #(return [pi, 0])

    print "result: ",ik_body(M, Tsd, thetalist, blist, e_theta, e_v)

