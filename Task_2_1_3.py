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

def build_B_matrix(theta,dt):
    B = np.array([[m.cos(theta)*dt/2,m.cos(theta)*dt/2],
                  [m.sin(theta)*dt/2,m.sin(theta)*dt/2],
                  [dt/r,-dt/r]])
    return B

C = np.array([[1,0,0],
              [0,1,0],
              [0,0,1]])
args = anki_vector.util.parse_command_args()

P_posterior = np.array([[100,0,0],
                        [0,100,0],
                        [0,0,100]])
trace_P = [float(P_posterior[0,0])+float(P_posterior[1,1])+float(P_posterior[2,2])]
#process
Q = np.array([[30,0,0],
              [0,30,0],
              [0,0,30]])
#sensor
R = np.array([[5,0,0],
              [0,5,0],
              [0,0,1e-8]])
print(R[0][0])
r= 24#mm
L= 48#mm
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

with anki_vector.Robot(args.serial) as robot:
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose) 
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    robot.motors.set_wheel_motors(100,100)
    time.sleep(0.1) 
    v1_k_1 = 100
    v2_k_1 = 100
    t = [0.0]
    dT = 0.0
    o_x = robot.pose.position.x
    o_y = robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.radians
    Xe_k_minus = np.array([[500],
                           [500],
                           [0.5]]) 
    print('//////////')
    start_time = time.time()
    last_time = start_time
    while True and dT<10.005:
        time.sleep(0.01)
        t.append(time.time()-start_time)
        theta = robot.pose.rotation.angle_z.radians - o_z
        v1=100
        v2=100
        robot.motors.set_wheel_motors(v2,v1)#left right
        #compute the value
        time_now = time.time()
        d_time = time_now -last_time
        print('dt',d_time)
        dT = time.time()-start_time
        B = build_B_matrix(Xe_k_minus[2],d_time)
        print('angle',Xe_k_minus[2])
        print('B',B)
        u = np.array([[v1_k_1],
                      [v2_k_1]])
        Xd_k_plus = np.dot(A,Xe_k_minus) + np.dot(B,u)
        print(Xd_k_plus)
        print('B*u',np.dot(B,u))
        #measurement value
        aa = robot.pose.position.x-o_x +gaussian_distribution_generator(5)
        bb = robot.pose.position.y-o_y +gaussian_distribution_generator(5)
        cc = robot.pose.rotation.angle_z.radians-o_z +gaussian_distribution_generator(1e-8)
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
       # fig, axs = plt.subplots(1,2)
        plt.plot(t,x_m,label='measurement')
        plt.plot(t,x_dyna,label='dynamic value')
        plt.plot(t,x_est,label='EKF estimate value')
       # plt.legend()
        plt.title('Position of Vector by different value')
        plt.xlabel('Time s')
        plt.ylabel('X position mm')
        #plt.plot(t,x_m,label='EKF estiamte value')
       # plt.plot(t,x_dyna,label='dynamic value',linestyle='--')
       # plt.plot(t,x_m,label='measurement value',linestyle='-.')
       # plt.title('The yaw angle value of Vector by different value')
       # plt.xlabel('Time s')
        #plt.ylabel('The angle value -- rad')
        #plt.legend()
       # plt.plot(t,trace_P,label='trace of Corvaiance matrix P')
      #  plt.title('The trace of Corvaiance matrix P')
       # plt.xlabel('Time s')
       # axs[1].legend()
        plt.legend()
        plt.show()