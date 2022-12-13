#!/usr/bin/env python3
# Task 3 update
import time
import anki_vector
from anki_vector.util import degrees, distance_mm, speed_mmps,Pose
import matplotlib.pyplot as plt
import math as m
##def
def find_nearly_points(current_x,current_y):
    d_n = [0.0]*N
    for i in range(N):
        dx = way_x[i]-current_x
        dy = way_y[i]-current_y
        d_n[i] = (dx**2 + dy**2)**0.5
    c = min(d_n)
    for j in range(N):
        if abs(c-d_n[j])<0.1:
            return j,d_n[j]

def find_next_goal(k):
    if k<N-4:
        return way_x[k+3],way_y[k+3]
    else:
        return way_x[N-1],way_y[N-1]

##=====================
angle_velocity = []
N=51
way_x = [0.0]*N
way_y = [0.0]*N
way_angle = [0.0]*N
for i in range(len(way_x)):
    way_x[i]=i*(800/N)
    way_y[i]=200*(m.sin((way_x[i])*2*m.pi/800))
    way_angle[i]=m.pi/2*m.cos(way_x[i]*m.pi/800)*180/m.pi
x_length = len(way_x)
y_length = len(way_y)
goal_x = way_x[1]
goal_y = way_y[1]
p_distance = (goal_x**2 + goal_y**2 )**0.5

args = anki_vector.util.parse_command_args()
k_rou = 4
k_aph = 4
r= 24#mm
L= 48#mm
d_distance = p_distance
print('d',d_distance)
z_angle = [0.0]
with anki_vector.Robot(args.serial) as robot:
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees
    time.sleep(1)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees
    time.sleep(1)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees
    print(o_x)
    print(o_y)
    print(o_z)
    time.sleep(1)
    z_gyro = []
    z_gyro.append(robot.gyro.z)
    t = 0
    count_i=1
    time_line =[]
    time_line.append(0.0)
    start_time = time.time()
    time_last = start_time
    cal_angle = [0.0]
    count_num =0 
    while True:
        if (robot.pose.position.x-o_x)**2+(robot.pose.position.y-o_y)**2>=(800**2-1):
            break
        time_now = time.time()
        d_time = time_now-time_last
        time_line.append(time.time()-start_time)
        cal_angle.append(robot.gyro.z*180/3.141592653589*d_time+cal_angle[count_num])
        i_points = find_nearly_points(robot.pose.position.x-o_x,robot.pose.position.y-o_y)
        [goal_x,goal_y] = find_next_goal(i_points[0])
        time.sleep(0.01)
        d_x = goal_x-(robot.pose.position.x-o_x)
        d_y = goal_y-(robot.pose.position.y - o_y)
        d_distance = m.sqrt(d_x**2 + d_y**2)
        vel = k_rou*d_distance
        theta = robot.pose.rotation.angle_z.degrees - o_z
        z_angle.append(theta)
        beta = (m.atan(d_y/d_x))*180/m.pi
        alpha = beta - theta
        #print('gyro',robot.gyro.z*180/3.141592653589)
        z_gyro.append(robot.gyro.z*180/3.141592653589)
  #      print('current angle',theta)
  #      print('beta',beta)
 #       print('alpha',alpha)
 #       print('d',d_distance)
  #      print('x',robot.pose.position.x-o_x)
  #      print('y',robot.pose.position.y-o_y)
  #      print("goal x",goal_x)
 #       print("goal y",goal_y)
        
        time_last = time_now
        count_num += 1
        if vel >100:
            vel = 100
        omiga = k_aph*alpha
        omiga = omiga*m.pi/180
        if omiga>=0:
            v1 = (L*omiga+2*vel)/2
            v2 = 2*vel-v1
        else:
            o_omiga = abs(omiga)
            v2 = (L*o_omiga+2*vel)/2
            v1 = 2*(vel)-v2


        robot.motors.set_wheel_motors(v2,v1)#left right
       # print("d",d_distance)
       # print("v1",v1)
       # print("v2",v2)
       # print("omiga",omiga)

    robot.motors.set_wheel_motors(0,0)
    time.sleep(1)
    print(z_angle)
    print(len(time_line))
    print(len(z_gyro))
    print('asdf')
    if True:
        plt.plot(time_line,z_angle,label='z')
        plt.plot(time_line,cal_angle,label='cal')
        plt.legend()
        plt.show()