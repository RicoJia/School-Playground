#!/usr/bin/python
import math
import numpy as np

def SO2_2_theta(R):
    """
    Input R is SO2 Rotation matrix.
    Sample Input: [[cos(theta) -sin(theta)],
        [sin(theta) cos(theta)]]
        Output is the angle of R
    Sample output: theta = pi/2
    """
    R = np.array(R)
    theta = math.atan2(R[1,0], R[0,0])
    return theta

def theta_2_SO2(theta):
    """
    Input theta is a scalar vector
    Output: SO2 Rotation matrix
    """
    c = math.cos(theta)
    s = math.sin(theta)
    R = np.array([[c, -s],
                  [s, c]])
    return R

def hat(w):
    """
    Returns the hatted version of a scalar w. The result is in so(3)
    """
    w_hat = np.array([[0.0, -w],
                      [w, 0.0]])
    return w_hat


def twist_2_SE2(twist):
    """
    Input: twist in unit time [theta, x, y], which is screw x theta. screw is [1, vx, vy].
    Output: T matrix SE2
        T = np.array([[R, P],
                       [0, 1]])
    """
    theta = twist[0]
    R = theta_2_SO2(theta)
    x = twist[1]
    y = twist[2]

    if theta == 0:
        G_theta_v = np.array([[x], [y]])
    else:
        vx = x/theta
        vy = y/theta
        R = theta_2_SO2(theta)
        w = 1 # this is the "omega axis "
        w_hat = hat(w)
        G_theta = np.identity(2)*theta + (1.0 - np.cos(theta))*w_hat + (theta - np.sin(theta))* w_hat.dot(w_hat)
        G_theta_v = G_theta.dot(np.array([[vx], [vy]]))

    T = np.vstack([np.hstack([R, G_theta_v]), np.array([[0, 0, 1]]) ])
    return T


def SE2_2_twist(T):
    """
    Input: T matrix SE2
            T = np.array([[R, P],
                       [0, 1]])
    Output: twist in unit time [theta, x, y], which is screw x theta. screw is [1, vx, vy].
    """
    T = np.copy(T)
    R= T[: 2, : 2]
    p = T[:2, -1]
    theta = SO2_2_theta(R)
    if abs(theta)<1e-6:
        xy = p
    else:
        w = 1 #this is the omega axis
        w_hat = hat(w)
        G_minus1 = 1/theta * np.identity(2) - 1.0/2.0*w_hat + ( 1/theta - 1.0/2.0*(1.0/np.tan(theta/2.0)))*w_hat.dot(w_hat)
        v = G_minus1.dot(np.array([[p[0]], [p[1]]]))
        xy = np.array([v[0], v[1]])* theta
        # print "xy: ", xy

    twist = np.append(theta, xy)
    return twist

def get_adjoint(T):
    """
    Returns the adjoint of a given SE(2) matrix
    """
    T = np.copy(T)
    R= T[: 2, : 2]
    p = T[:2, -1]
    c = R[0, 0]
    s = R[1, 0]
    x = p[0]
    y = p[1]
    Adj = np.array([[1.0, 0.0, 0.0],
                    [y, c, -s],
                    [-x, s, c]])
    return Adj


if __name__=='__main__':

    v1 = [0, 0.0, 0.0]
    v2 = [np.pi/2, 0.0, 0.0]
    v3 = [- np.pi/2, 0.0, 0.0]
    v4 = [1.6, 14.0, 2.0]
    v5 = [0.3, 1.1, 2.2]

    print (v1, SE2_2_twist(twist_2_SE2(v1)) )
    print (v2, SE2_2_twist(twist_2_SE2(v2)) )
    print (v3, SE2_2_twist(twist_2_SE2(v3)) )
    print (v4, SE2_2_twist(twist_2_SE2(v4)) )
    print (v5, SE2_2_twist(twist_2_SE2(v5)) )
