import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath
from spatialmath import SE3
import matplotlib

a1 = float(input("a1 = "))*100
a2 = float(input("a2 = "))*100
a3 = float(input("a3 = "))*100

def mm_m(a):
    m = 100 #; meter = 1000mm
    return a/m

a1 = mm_m(a1)
a2 = mm_m(a2)
a3 = mm_m(a3)


# Create links
#robot variable =  DHrobot([RevoluteDH,(d,r,alpha,offset,qlim)])
#robot variable =  DHrobot([PrismaticDH,(d=0,r,alpha,offset=d,qlim)])

Articulated = DHRobot([
    RevoluteDH(a1,0,(90.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a2,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    RevoluteDH(0,a3,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2])
],name='Articulated')

print(Articulated)

## Path and Trajectory Planning

# degrees to radian converter
def deg_to_rad(T):
    return (T/180.0)*np.pi

## q Planned Paths
# for Cylindrical Jonit Variables = ([T1,d2,d3])

q0 = np.array([0,0,0])

q1 = np.array([deg_to_rad(0),
                deg_to_rad(-30),
                deg_to_rad(-30)])  #pick1

q2 = np.array([deg_to_rad(0),
                deg_to_rad(0),
                deg_to_rad(0)])  #return1

q3 = np.array([deg_to_rad(90),
                deg_to_rad(0),
                deg_to_rad(0)])  #revolve1

q4 = np.array([deg_to_rad(90),
                deg_to_rad(-30),
                deg_to_rad(-30)])  #place1


q5 = np.array([deg_to_rad(90),
                deg_to_rad(0),
                deg_to_rad(0)]) #return 2

q6 = np.array([deg_to_rad(180),
                deg_to_rad(0),
                deg_to_rad(0)]) #revolve2

q7 = np.array([deg_to_rad(180),
                deg_to_rad(-30),
                deg_to_rad(-30)]) #pick 3

q8 = np.array([deg_to_rad(180),
                deg_to_rad(0),
                deg_to_rad(0)]) #return3

q9 = np.array([deg_to_rad(270),
                deg_to_rad(0),
                deg_to_rad(0)]) #revolve3

q10 = np.array([deg_to_rad(270),
                deg_to_rad(-30),
                deg_to_rad(-30)]) #place3

q11 = np.array([deg_to_rad(270),
                 deg_to_rad(0),
                 deg_to_rad(0)]) #return4

q12 = np.array([deg_to_rad(360),
                deg_to_rad(0),
                deg_to_rad(0)]) #revolve4

q13 = np.array([deg_to_rad(360),
                deg_to_rad(-30),
                deg_to_rad(-30)]) # place 4 


# planned Trajectories
traj1 = rtb.jtraj(q0,q1,20)
traj2 = rtb.jtraj(q1,q2,20)
traj3 = rtb.jtraj(q2,q3,20)
traj4 = rtb.jtraj(q3,q4,20)
traj5 = rtb.jtraj(q4,q5,20)
traj6 = rtb.jtraj(q5,q6,20)
traj7 = rtb.jtraj(q6,q7,20)
traj8 = rtb.jtraj(q7,q8,20)
traj9 = rtb.jtraj(q8,q9,20)
traj10 = rtb.jtraj(q9,q10,20)
traj11 = rtb.jtraj(q10,q11,20)
traj12 = rtb.jtraj(q11,q12,20)
traj13 = rtb.jtraj(q12,q13,20)
traj14 = rtb.jtraj(q13,q0,20)

#plot scale
x1 = -50
x2 = 50
y1 = -50
y2 = 50
z1 = 0
z2 = 50

# Path and Trajectory plotting
Articulated.plot(traj1.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj2.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj3.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj4.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj5.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj6.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj7.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj8.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj9.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj10.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj11.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj12.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj13.q,limits=[x1, x2, y1, y2, z1, z2])
Articulated.plot(traj14.q,limits=[x1, x2, y1, y2, z1, z2],block=True)



