# Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024
![1](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/79f6cf39-1254-43da-9b8b-19182c8d54f2)
![2](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/82f6e5e7-b38c-42c5-bc37-990e5ade2633)
![3](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/93983368-8035-4e90-b6ab-bd541f46ab16)
**LINK FOR TASK 2:**
https://drive.google.com/file/d/1VBuQTJ9nbtTNi2ZxQbCpKRM8b81p4c8o/view?usp=drive_link

**LINK FOR TASK 4:**
https://drive.google.com/file/d/136p0FQQEMhvFcT6H3m3JVeorJDbAsUvk/view?usp=drive_link

**LINK FOR TASK 5:**
https://drive.google.com/file/d/1RvCZi8VDp7HcbdiaaJg9v4KS0fdVEMjE/view?usp=drive_link

![4](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/7c4d7206-198e-45d2-94f2-3c6790923b80)
![5](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/d89fb2bc-0983-466e-91ce-d83258970132)
![6](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/a5d57806-a9a1-4003-a32d-e8477acd238a)
![7](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/2c6f0a96-a0ff-44a8-8361-d9d8de20af5b)
![8](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/9a01c777-6db1-4ab9-b085-efbc245e5a99)
![9](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/2491b116-2638-4a44-b558-77feb3e789ea)
![10](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/595d2886-0d62-4177-bd45-6675b2c8e317)
![11](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/83acc54a-b3db-47fb-a1be-340a62ed1593)
![13](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/a8fe3021-9ad7-454e-a756-92a9a9b180f7)
![14](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/18dd7f44-5bba-488d-ac2a-489ca51e9f82)
![15](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/9be6e76c-3ecc-46d4-9ccc-878cefbb1ffb)
![16](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/2b3378e3-eda5-4b9e-8952-c4066ce386d8)
![17](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/d7ac14cc-d1f5-4324-ac24-2986755f67b5)
![18](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/3522a97b-ecab-49ba-a1c4-80392d3f6bfe)
![JACOBIAN MATRIX AND TRAJECTORY PLANNING_ROBOTICS 2 FINAL PROJECT](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/9e020c9e-1b86-4441-ab66-0826e9025288)
![27](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/1ee2de74-04bb-4699-835c-13b22fd4fbda)
![28](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/4b3f7086-c028-44bc-bd94-45d51bb3efd6)
![29](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/835b4078-cb3c-4c27-adc9-311157640a44)
![19](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/7fe2e549-34b1-49b6-98fb-77aceaceaa56)
![20](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/5f1e7272-14e3-4281-80bd-c65f19efe711)
![21](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/8dd77c02-b61a-4704-b4d9-85636f2ef06c)
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
![22](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/cd5f6eb0-a8c9-4081-bdde-18be44d120f3)

![24](https://github.com/icecreamperson/Robotics2_JacobianandPT_Group8_ArticulatedManipulator_2024/assets/157493649/218da746-3142-4e20-a5bd-589e0b5b4b50)
