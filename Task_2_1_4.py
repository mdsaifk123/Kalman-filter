#!/usr/bin/env python3
# Task 2
import time
import anki_vector
from anki_vector.util import degrees, distance_mm, speed_mmps,Pose
import matplotlib.pyplot as plt
import math as m
import numpy as np

def gaussian_distribution_generator(var):
    return np.random.normal(loc=0.0, scale=var**0.5, size=None)

#set B matrix
def build_B_matrix(theta,dt):
    B = np.array([[m.cos(theta)*dt/2,m.cos(theta)*dt/2],
                  [m.sin(theta)*dt/2,m.sin(theta)*dt/2],
                  [dt/r,-dt/r]])
    return B

#set C matrix
C = np.array([[1,0,0],
              [0,1,0],
              [0,0,1]])

#initial guess of X variable
Xe_k_minus = np.array([[0.0],[0.0],[0.0]])
#set 1meter,1meter goal position
goal_x = 1000
goal_y = 1000
p_distance = (goal_x**2 + goal_y**2 )**0.5
args = anki_vector.util.parse_command_args()
#control gain
k_rou = 3
k_aph = 3
accx = []
accl_x = open('acc_x.txt','w')
#acce y
accy = []
accl_y = open('acc_y.txt','w')
accz = []
accl_z = open('acc_y.txt','w')
#position x
current_x = []
c_x = open('c_x.txt','w')
#position y
current_y=[]
c_y = open('c_y.txt','w')
#angle z
z_angle = []
c_z = open('angle_z.txt','w')
#v2
left_v = []
left_vel = open('left_vel.txt','w')
#v1
right_v=[]
right_vel = open('right_vel.txt','w')
#initial P
P_posterior = np.array([[100,0,0],
                        [0,100,0],
                        [0,0,100]])
trace_P = [float(P_posterior[0,0])+float(P_posterior[1,1])+float(P_posterior[2,2])]
#process
Q = np.array([[10.41, 0,0],
              [0, 10.41,0],
              [0,0,10.41]])
#sensor
R = np.array([[3,0,0],
              [0,3,0],
              [0,0,0.1]])
r= 24#mm half length of two wheel distance
L= 48#mm distance of two wheel
#find A matrix
A = np.array([[1,0,0],
              [0,1,0],
              [0,0,1]])
x_est=[0.0]
y_est=[0.0]
z_ang_est=[0.0]
#dynamic
x_dyna = [0.0]
y_dyna =[0.0]
z_ang_dyna = [0.0]
#meansure
x_m = [0.0]
y_m = [0.0]
z_m =[0.0]
d_distance = p_distance
with anki_vector.Robot(args.serial) as robot:
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.radians
    Xe_k_minus = np.array([[robot.pose.position.x-o_x],
                           [robot.pose.position.y-o_y],
                           [robot.pose.rotation.angle_z.radians-o_z]])
    #velocity k - 1
    v1_k_1 = 0
    v2_k_1 = 0
    o_angz = o_z
    t = [0.0]
    v1_list = [0.0]
    v2_list = [0.0]
    diff_time = [0.0]
    print('//////////')
    time.sleep(5)
    start_time = time.time()
    last_time = 0.0
    while True and not(d_distance<8):
        time.sleep(0.01)
        t.append(time.time()-start_time)      #add time line 
        print('est x',Xe_k_minus[0])
        print('est y',Xe_k_minus[1])
        d_x = goal_x-(Xe_k_minus[0] - o_x)    #update different of x
        d_y = goal_y-(Xe_k_minus[1] - o_y)    #update different of y
        d_distance = m.sqrt(d_x**2 + d_y**2)  #update distance of vector and goal positiob
        vel = k_rou*d_distance
        theta = robot.pose.rotation.angle_z.radians - o_angz
        beta = (m.atan(d_y/d_x))
        print('beta',beta)
        alpha = beta - theta
        print('alpha',alpha)
        #make sure velocity is not too high
        if vel > 80: 
            vel = 80
        omiga = k_aph*alpha
        #make sure velocity is not too low
        if vel <25:
            vel =vel+ 20
        if omiga>=0:
            v1 = (L*omiga+2*vel)/2
            v2 = 2*vel-v1
        else:
            o_omiga = abs(omiga)
            v2 = (L*o_omiga+2*vel)/2
            v1 = 2*(vel)-v2
        robot.motors.set_wheel_motors(v2,v1)#left right
        #compute the value
        time_now = time.time()
        d_time = time_now -last_time
        diff_time.append(d_time)
        v1_list.append(Xe_k_minus[0])
        v2_list.append(Xe_k_minus[1])
        #build B matrix EKF
        B = build_B_matrix(Xe_k_minus[2],d_time)
        #B = build_B_matrix(0.0,d_time)
        print('B',B)
        #update last state velocity
        u = np.array([[v1_k_1],
                      [v2_k_1]])
        #dynamic value
        Xd_k_plus = np.dot(A,Xe_k_minus) + np.dot(B,u)
        print(Xd_k_plus)
        print('B*u',np.dot(B,u))
        #measurement value
        aa = robot.pose.position.x-o_x +gaussian_distribution_generator(3)
        bb = robot.pose.position.y-o_y +gaussian_distribution_generator(3)
        cc = robot.pose.rotation.angle_z.radians-o_angz +gaussian_distribution_generator(1e-8)
        X_measure = np.array([[aa],
                              [bb],
                              [cc]])
        #filter
        P_prior_1 = np.dot(A, P_posterior)
        P_prior = np.dot(P_prior_1, A.T) + Q
        #K----------
        k1 = np.dot(P_prior, C.T)
        k2 = np.dot(np.dot(C, P_prior), C.T) + R 
        Kk = np.dot(k1,np.linalg.inv(k2))
        print('kk',Kk)
        Xe_k =  Xd_k_plus + np.dot(Kk,(X_measure - np.dot(C,Xd_k_plus)))
        # update
        last_time = time_now
        Xe_k_minus = Xe_k
        trace_P.append(float(P_prior[0,0])+float(P_prior[1,1])+float(P_prior[2,2]))
        P_posterior_1 = np.eye(3) - np.dot(Kk, C)
        P_posterior = np.dot(P_posterior_1, P_prior)
        #update to the list to prepare plot
        x_est.append(Xe_k[0])
        y_est.append(Xe_k[1])
        z_ang_est.append(Xe_k[2])
        x_dyna.append(Xd_k_plus[0])
        y_dyna.append(Xd_k_plus[1])
        z_ang_dyna.append(Xd_k_plus[2]) 
        x_m.append(aa) 
        y_m.append(bb) 
        z_m.append(cc) 
        v1_k_1 = v1
        v2_k_1 = v2
    robot.motors.stop_all_motors
    robot.motors.set_wheel_motors(0,0)

    if True:
        f1 = plt.figure()
        plt.plot(x_m,y_m,label='measurement')
        plt.plot(x_dyna,y_dyna,label='dynamic value')
        plt.plot(x_est,y_est,label='KF estimate value')
        plt.legend()
        plt.title('Position of Vector by different value')
        plt.xlabel('X position mm')
        plt.ylabel('Y position mm')
        f2 = plt.figure()
        plt.plot(t,z_ang_est,label='KF estiamte value')
        plt.plot(t,z_ang_dyna,label='dynamic value')
        plt.plot(t,z_m,label='measurement value')
        plt.title('The yaw angle value of Vector by different value')
        plt.xlabel('Time s')
        plt.ylabel('The angle value -- rad')
        plt.legend()
        f3 = plt.figure()
        plt.plot(t,trace_P,label='trace of Corvaiance matrix P')
        plt.title('The trace of Corvaiance matrix P')
        plt.xlabel('Time s')
        plt.legend()
        f4 = plt.figure()
        plt.plot(t,x_est,label='KF estiamte value')
        plt.plot(t,x_dyna,label='dynamic value')
        plt.plot(t,x_m,label='measurement value')
        plt.title('The position of x by different value')
        plt.xlabel('Time s')
        plt.ylabel('The position of x value -- mm')
        plt.legend()
        f5 = plt.figure()
        plt.plot(t,y_est,label='KF estiamte value')
        plt.plot(t,y_dyna,label='dynamic value')
        plt.plot(t,y_m,label='measurement value')
        plt.title('The position of y by different value')
        plt.xlabel('Time s')
        plt.ylabel('The position of y value -- mm')
        plt.legend()
        f6 = plt.figure()#dT
        plt.plot(t,diff_time,label='diff time')
        plt.title('different time')
        plt.xlabel('Time s')
        plt.ylabel('The position of y value -- mm')
        plt.legend()
        plt.show()