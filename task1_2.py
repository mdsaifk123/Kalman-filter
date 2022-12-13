#!/usr/bin/env python3
import time
import anki_vector
from anki_vector.util import degrees, distance_mm, speed_mmps,Pose
import matplotlib.pyplot as plt
import math as m
import numpy as np
def gaussian_distribution_generator(var):
    return np.random.normal(loc=0.0, scale=var**0.5, size=None)
args = anki_vector.util.parse_command_args()
r= 12#mm
L= 48#mm
t_time = [0.0]
x_measurement = [0.0]
x_estimate = [0.0]
x_model =[0.0] 
#process
Q =30 #np.array([[10.41, 0],
             # [0, 0]])

#sensor
R =3 #np.array([[0.01,0],
             # [0, 0]])
C = np.array([[1.0, 0]])
##
#find Q process noise
#find R sensor noise
#build A matrix
#[1 dt]
#[0 1]
#find P_k_minus 
#find K_k = P_k_mins*C'(C*P_k_minus*C'+R)^-1 
#find x_k_model = A*x_estimate
#find 
#x_k_estimate = x_k_model + Kk(meansure - C*x_k_model)
#P_k_plus=(_matrix-KkC)Pk_minus

I_matrix = np.eye(2)
X0 = np.array([[0],
               [100]])
x_est_old = X0
P = np.array([[50, 0],
              [0, 50]])
dT = 0
P_old = P

with anki_vector.Robot(args.serial) as robot:
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x = robot.pose.position.x
    o_y = robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees 
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose) 
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose) 
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose) 
    print(o_x)
    print(o_y)
    print(o_z)
    print('=======')
    robot.motors.set_wheel_motors(100,100)
    time.sleep(2)
    Kk_gain = [0.0]
    trace_P = [(P_old[0,0]+P_old[1,1])]
    print(trace_P)
    o_x = robot.pose.position.x
    time_start = time.time()
    time_stamp_last=time_start
    while True and dT<10:
        if dT > 5 and R != 6:
            R = R*2
        print('x_measure',robot.pose.position.x)
        time.sleep(0.01)
        time_stamp = time.time()
        dT = time.time()-time_start
        t_time.append(dT)
        delt_time = time_stamp - time_stamp_last
        A = np.array([[1, delt_time],
              [0, 1]])
        X_k_plus = np.dot(A,x_est_old)
        y_measurement = robot.pose.position.x-o_x +gaussian_distribution_generator(R)
        #P
        P_prior_1 = np.dot(A, P_old)
        P_prior = np.dot(P_prior_1, A.T) + Q
        print('P_prior',P_prior)
        #
        k1 = np.dot(P_prior, C.T)
        k2 = np.dot(np.dot(C, P_prior), C.T) + R
        print('k1',k1)
        print('k2',k2)
        Kk = np.dot(k1, np.linalg.inv(k2))
        X_k_esitmate = X_k_plus + Kk*(y_measurement-np.dot(C,X_k_plus))
        print('esitmate',X_k_esitmate)
        print('X_k_plus',X_k_plus)
        print('measurement',y_measurement)
        print('model',X_k_esitmate[0])
        print('time',delt_time)
        P_old_1 = I_matrix - np.dot(Kk, C)
        P_old = np.dot(P_old_1, P_prior)
        x_measurement.append(y_measurement)
        x_estimate.append(float(X_k_esitmate[0]))
        x_model.append(float(X_k_plus[0]))
        print('P',P_old)
        print('========')
        Kk_gain.append(Kk)
        x_est_old = X_k_esitmate
        trace_P.append((P_old[0,0]+P_old[1,1]))
        time_stamp_last = time_stamp
    print(R)
    print(robot.pose.position.x-o_x)
    print(dT)
    robot.motors.set_wheel_motors(0,0)
    time.sleep(1)
    print('estimate',x_estimate)
    print('model',x_model)
    print('measurement',x_measurement)
    print('/./././././././././././')
    if True:
        fig, axs = plt.subplots(1,2)
        axs[0].plot(t_time,x_measurement,label="measurement value", linewidth=0.5)
        axs[0].plot(t_time,x_estimate,label="estimate value", linewidth=2)
        axs[0].plot(t_time,x_model,label="dynamic value", linewidth=0.5)
        axs[0].set_xlabel('Time s')
        axs[0].set_ylabel('Position mm')
        axs[0].legend()
        axs[0].set_title("Position")
        axs[1].plot(t_time,trace_P,label="P")
        axs[1].set_xlabel('Time s')
        axs[1].set_title("Trace of Covariance matrix P")
        axs[1].legend()
        plt.show()
#    for i in average_v:
#        average_vel.write(str(i))
#        average_vel.write('\n')