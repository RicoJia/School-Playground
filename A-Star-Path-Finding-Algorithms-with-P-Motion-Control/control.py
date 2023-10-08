import numpy as np
import math
import matplotlib.pyplot as plt
import time


def controller(start,end,start_vel,delta_t, add_noise=False):
    """ 
    Purpose: taking turtle from point A to point B
    Inputs: start is [x,y,theta], end [x,y,theta], start_vel [v,w] 
    output: updated [v,w]"""
    dist_e = end-start
    kx = 0.01
    ky = 0.01
    heading = start[2]
    #dist_e_thre = 0.01
    
    av_thre = 0.288
    aw_thre = 5.579
    # uv is [uvx,uvy]
    uv = np.array([kx*dist_e[0],ky*dist_e[1]])
    #uw is scaler
    uw = (math.atan2(uv[1],uv[0])-heading)/delta_t 
   
   #av is commanded acceleration.
    sign_av = np.sign(uv.dot(uv)-start_vel[0]**2)
    av = np.sqrt(abs((uv.dot(uv)-start_vel[0]**2)))/delta_t
    aw = (uw-start_vel[1])/delta_t
    sign_aw = np.sign(aw)


    #uv = start_vel + 
    uv_r = sign_av*min(av_thre,abs(av))*delta_t+start_vel[0]
    uw_r = sign_aw*min(abs(aw),aw_thre)*delta_t+start_vel[1]
    new_vel = np.array([uv_r,uw_r])

#    if add_noise:
#        mu_v, sigma_v = 0, 0.1
#        s_v = np.random.normal(mu_v,sigma_v,1)
#        mu_w, sigma_w = 0,1
#        s_w = np.random.normal(mu_w, sigma_w,1)
#        s = np.append( s_v, s_w )
#        new_vel = new_vel + s
  
    return new_vel

def motion_model(start,vel,delta_t,add_noise = False):
    """ purpose: updated position of turtle in delta_t
        inputs: start [x,y,theta],vel[v,w], delta_t
        output: updated[x,y,theta]
    """
    v = vel[0]
    w = vel[1]
    x = start[0]
    y = start[1]
    theta = start[2]
    xdot = v*np.cos(theta)
    ydot = v*np.sin(theta)
    new_pos = np.array([x+xdot*delta_t,y+ydot*delta_t,theta+w*delta_t])
    
    if add_noise:
        mu_v, sigma_v = 0, 0.001
        s_v = np.random.normal(mu_v,sigma_v,2)
        mu_w, sigma_w = 0,0.005
        s_w = np.random.normal(mu_w, sigma_w,1)
        s = np.append( s_v, s_w )
        new_pos = new_pos + s

    return new_pos


if __name__=="__main__":
    start = np.array([1,3,0])
    end = np.array([0,2,0])
    vel = np.array([0,0])
    T = 60
    delta_t = 0.02
    x_vec = []
    y_vec = []
    for i in np.arange(T/delta_t):
        vel = controller(start,end,vel,delta_t)
        start = motion_model(start,vel,delta_t)
        print "position is: ", start
        x_vec.append(start[0])
        y_vec.append(start[1])
    plt.plot(x_vec,y_vec)
    
    plt.show()

        
