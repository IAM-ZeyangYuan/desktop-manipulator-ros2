# -*- coding: utf-8 -*-
"""
Created on Fri Jan 23 20:27:12 2026

@author: ZeyangYuan
"""
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.interpolate import CubicSpline


#
#    %matplotlib opens an interactive plot window

#may be get a somewhat spiral trajectory

from pyomo.environ import (
    ConcreteModel, Var, Objective, Constraint, ConstraintList, RangeSet, Reals, SolverFactory, value, summation, minimize
)

plt.rcParams.update({
    'figure.figsize': (6, 4),
    'figure.dpi': 100,
    'font.size': 18,
    'axes.linewidth': 1.0,
    'lines.linewidth': 3,
    'grid.linestyle': ':',
    'grid.color': 'gray',
    'grid.alpha': 0.5,
    
    'font.weight': 'bold',          # <-- global text
    'axes.labelweight': 'bold',      # <-- x/y labels
    'axes.titleweight': 'bold'      # <-- titles
})
plt.rcParams['mathtext.default'] = 'bf'

plt.close('all')

#fixed parameters (unit: meter)
L1 = 35
L2 = 25
L3 = 2
L4 = 10

#3 lists for the xyz of our points
x = []
y = []
z = []

#honestly, i can just alternate between those numbers cuz theta1 doesnt really affect the z axis
#having the same consecutive points should be fine, just use hyperbolic blend or whatever

# nump = 4
# Theta1 = [-np.pi/4, np.pi/4, np.pi/8, -np.pi/8]   
# Theta2 = [-np.pi/3, -np.pi/3, np.arctan(-4), np.arctan(-4)]
# Theta3 = [-np.pi/6, np.pi/6, np.pi/3, -np.pi/3]
# D4 = [0.5, 0.5, 3, 3] 


nump = 12
Theta1 = [-np.pi/4, np.pi/4, np.pi/8, -np.pi/8,
          -np.pi/4, np.pi/4, np.pi/8, -np.pi/8,
          -np.pi/4, np.pi/4, np.pi/8, -np.pi/8] 
Theta2 = [-np.pi/6, -np.pi/6, -np.arctan(3/4), -np.arctan(3/4),
          np.arctan(5) - np.pi, np.arctan(5) - np.pi, np.arctan(6) - np.pi, np.arctan(6) - np.pi,
          -np.pi*5/6, -np.pi*5/6, np.arctan(3/4)-np.pi, np.arctan(3/4)-np.pi]
Theta3 = [-np.arccos((21*np.sqrt(3)-35)/5), np.arccos((21*np.sqrt(3)-35)/5), np.arccos(1/30),-np.arccos(1/30),
          np.arccos(-16/25), -np.arccos(-16/25), 2/3*np.pi, -2/3*np.pi,
          np.arccos((21*np.sqrt(3)-35)/5)-np.pi, -np.arccos((21*np.sqrt(3)-35)/5)+np.pi, -np.arccos(1/30)+np.pi,np.arccos(1/30)-np.pi]

#Theta3 = [abs(baga) for baga in Theta31]

D4 = [14,14,15,15,
      9,9,8,8,
      14,14,15,15] 



for i in range(nump):
    theta1 = Theta1[i]
    theta2 = Theta2[i]
    theta3 = Theta3[i]
    d4 = D4[i]
    
    
    T01 = np.array([[np.cos(theta1),-np.sin(theta1),0,0], 
                    [np.sin(theta1),np.cos(theta1),0,0], 
                    [0,0,1,L1], 
                    [0,0,0,1]])

    T12 = np.array([[np.cos(theta2),-np.sin(theta2),0,L2], 
                    [0,0,1,0], 
                    [-np.sin(theta2),-np.cos(theta2),0,0], 
                    [0,0,0,1]])

    T23 = np.array([[np.cos(theta3),-np.sin(theta3),0,0], 
                    [0,0,1,L3], 
                    [-np.sin(theta3),-np.cos(theta3),0,0], 
                    [0,0,0,1]])


    T34 = np.array([[1,0,0,L4],
                    [0,1,0,0], 
                    [0,0,1,d4], 
                    [0,0,0,1]])

    #the end effector in frame{4}
    P_in4 = np.array([-L4/2,0,L4/2,1])#!!!

    #the end effector in frame{0}
    P_in0 = T01 @ T12 @ T23 @ T34 @ P_in4
    x.append(P_in0[0])
    y.append(P_in0[1])
    z.append(P_in0[2])
                
                
                
    

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')


workspace = ax.plot(x,y,z)




ax.set_xlim(-50, 50)  
ax.set_ylim(-50, 50)
ax.set_zlim(0, 60)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()





# define a piecewise function for the parabolic blend
def pblend1(y0,y1,t0,delt,t):
    a0 = y0+2*(y1-y0)*t0**2/delt**2
    a1 = -4*(y1-y0)*t0/delt**2
    a2 = 2*(y1-y0)/delt**2
    
    return (a0+a1*t+a2*t**2, a1+2*a2*t, 2*a2)

def pblend2(y0,y1,t0,delt,t):
    a0 = y0+2*(y1-y0)*t0**2/delt**2
    a1 = -4*(y1-y0)*t0/delt**2
    a2 = 2*(y1-y0)/delt**2
    
    
    return (y0+y1-pblend1(y0,y1,t0,delt,2*t0+delt-t)[0], 
            a1+2*a2*(2*t0+delt-t),
            -2*a2)

def pbblend(y0,y1,t0,delt,t):
    if t0 <= t <= t0+delt/2:
        return pblend1(y0,y1,t0,delt,t)[0]
    if t0+delt/2 < t <= t0+delt:
        return pblend2(y0,y1,t0,delt,t)[0]
    
    
def vel(y0,y1,t0,delt,t):
    if t0 <= t <= t0+delt/2:
        return pblend1(y0,y1,t0,delt,t)[1]
    if t0+delt/2 < t <= t0+delt:
        return pblend2(y0,y1,t0,delt,t)[1]
    
def acc(y0,y1,t0,delt,t):
    if t0 <= t <= t0+delt/2:
        return pblend1(y0,y1,t0,delt,t)[2]
    if t0+delt/2 < t <= t0+delt:
        return pblend2(y0,y1,t0,delt,t)[2]




# trajectory planning

plt.close('all')

delta_t = 2.5 #seconds
t = np.linspace(0,11*delta_t,12)
pp = 100 #plot points in each segment


#Theta2
# creating 5+6 segments in the dumbest way

theta2_0 = [Theta2[0]]*(pp-1)
t1 = np.linspace(delta_t,2*delta_t,pp)
theta2_1 = []
theta2_2 = [Theta2[2]]*(pp-2)
t3 = np.linspace(3*delta_t,4*delta_t,pp)
theta2_3 = []
theta2_4 = [Theta2[4]]*(pp-2)
t5 = np.linspace(5*delta_t,6*delta_t,pp)
theta2_5 = []
theta2_6 = [Theta2[6]]*(pp-2)
t7 = np.linspace(7*delta_t,8*delta_t,pp)
theta2_7 = []
theta2_8 = [Theta2[8]]*(pp-2)
t9 = np.linspace(9*delta_t,10*delta_t,pp)
theta2_9 = []
theta2_10 = [Theta2[10]]*(pp-1)

for i in t1:
    theta2_1.append(pbblend(Theta2[1],Theta2[2],delta_t,delta_t,i))

for i in t3:
    theta2_3.append(pbblend(Theta2[3],Theta2[4],3*delta_t,delta_t,i))

for i in t5:
    theta2_5.append(pbblend(Theta2[5],Theta2[6],5*delta_t,delta_t,i))

for i in t7:
    theta2_7.append(pbblend(Theta2[7],Theta2[8],7*delta_t,delta_t,i))
    
for i in t9:
    theta2_9.append(pbblend(Theta2[9],Theta2[10],9*delta_t,delta_t,i))
    

comb2 = theta2_0 + theta2_1 + theta2_2 + theta2_3 + theta2_4 + theta2_5 + theta2_6 + theta2_7 + theta2_8 + theta2_9 + theta2_10
tall = np.linspace(0,11*delta_t,len(comb2))




#D4
d4_0 = [D4[0]]*(pp-1)

d4_1 = []
d4_2 = [D4[2]]*(pp-2)

d4_3 = []
d4_4 = [D4[4]]*(pp-2)

d4_5 = []
d4_6 = [D4[6]]*(pp-2)

d4_7 = []
d4_8 = [D4[8]]*(pp-2)

d4_9 = []
d4_10 = [D4[10]]*(pp-1)

for i in t1:
    d4_1.append(pbblend(D4[1],D4[2],delta_t,delta_t,i))

for i in t3:
    d4_3.append(pbblend(D4[3],D4[4],3*delta_t,delta_t,i))

for i in t5:
    d4_5.append(pbblend(D4[5],D4[6],5*delta_t,delta_t,i))

for i in t7:
    d4_7.append(pbblend(D4[7],D4[8],7*delta_t,delta_t,i))
    
for i in t9:
    d4_9.append(pbblend(D4[9],D4[10],9*delta_t,delta_t,i))
    

comb4 = d4_0 + d4_1 + d4_2 + d4_3 + d4_4 + d4_5 + d4_6 + d4_7 + d4_8 + d4_9 + d4_10


# theta1 and theta 3 using cubic spline


# Cubic spline with first derivative = 0 at both ends
cstheta1 = CubicSpline(t,Theta1, bc_type=((1, 0), (1, 0)))
cstheta3 = CubicSpline(t,Theta3, bc_type=((1, 0), (1, 0)))




comb1 = cstheta1(tall)
comb3 = cstheta3(tall)


#####
# cstheta2 = CubicSpline(t,Theta2, bc_type=((1, 0), (1, 0)))
# csd4 = CubicSpline(t,D4, bc_type=((1, 0), (1, 0)))

# comb2 = cstheta2(tall)
# comb4 = csd4(tall)
# ######


plt.figure(figsize=(8, 8))

plt.subplot(2, 2, 1)
plt.plot(tall, comb1, zorder=1)                 # line first (background)
plt.scatter(t, Theta1, c='red', zorder=2) 
plt.ylim(-np.pi, np.pi)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\theta_1(rad)$")
plt.xlim(0, delta_t * 11)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 




plt.subplot(2, 2, 2)
plt.plot(tall, comb2, zorder=1)                 # line first (background)
plt.scatter(t, Theta2, c='red', zorder=2) 
plt.ylim(-np.pi, np.pi)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\theta_2(rad)$")
plt.xlim(0, delta_t * 11)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 


plt.subplot(2, 2, 3)
plt.plot(tall, comb3, zorder=1)                 # line first (background)
plt.scatter(t, Theta3, c='red', zorder=2) 
plt.ylim(-np.pi, np.pi)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\theta_3(rad)$")
plt.xlim(0, delta_t * 11)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 

plt.subplot(2, 2, 4)
plt.plot(tall, comb4, zorder=1)                 # line first (background)
plt.scatter(t, D4, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$d_4(cm)$")
plt.xlim(0, delta_t * 11)
plt.ylim(5, 17)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([5,10,15]) 


 
plt.tight_layout()
plt.show()









#%% velocity in the joint space)

plt.close('all')
plt.figure(figsize=(8, 8))

t_indices = list(np.arange(0,11*(pp-1)+1,pp-1))

vel1 = np.gradient(comb1,tall)
acc1 = np.gradient(vel1,tall)
vel1_sampled = [vel1[i] for i in t_indices]
acc1_sampled = [acc1[i] for i in t_indices]

vel2 = np.gradient(comb2,tall)
acc2 = np.gradient(vel2,tall)
vel2_sampled = [vel2[i] for i in t_indices]
acc2_sampled = [acc2[i] for i in t_indices]

vel3 = np.gradient(comb3,tall)
acc3 = np.gradient(vel3,tall)
vel3_sampled = [vel3[i] for i in t_indices]
acc3_sampled = [acc3[i] for i in t_indices]

vel4 = np.gradient(comb4,tall)
acc4 = np.gradient(vel4,tall)
vel4_sampled = [vel4[i] for i in t_indices]
acc4_sampled = [acc4[i] for i in t_indices]

plt.subplot(2, 2, 1)
plt.plot(tall,vel1, zorder = 1)
#plt.scatter(t,vel1_sampled, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\dot{\theta}_1(rad/s)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 


plt.subplot(2, 2, 2)
plt.plot(tall,vel2, zorder = 1)
#plt.scatter(t,vel2_sampled, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\dot{\theta}_2(rad/s)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 


plt.subplot(2, 2, 3)
plt.plot(tall,vel3, zorder = 1)
#plt.scatter(t,vel3_sampled, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\dot{\theta}_3(rad/s)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 

plt.subplot(2, 2, 4)
plt.plot(tall,vel4, zorder = 1)
#plt.scatter(t,vel4_sampled, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\dot{d}_4(cm/s)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-5.2,5.2)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-5,-2.5, 0, 2.5, 5]) 



plt.tight_layout()
plt.show()

#%% acceleration (in the joint space)

plt.close('all')
plt.figure(figsize=(8, 8))

acc1 = np.gradient(vel1,tall)

acc2 = np.gradient(vel2,tall)

acc3 = np.gradient(vel3,tall)

acc4 = np.gradient(vel4,tall)

plt.subplot(2, 2, 1)
plt.plot(tall,acc1, zorder = 1)
#plt.scatter(t,acc1_sampled, c='red', zorder=2) 
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\ddot{\theta}_1(rad/s^2)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 


plt.subplot(2, 2, 2)
plt.plot(tall,acc2, zorder = 1)
#plt.scatter(t,acc2_sampled, c='red', zorder=2)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\ddot{\theta}_2(rad/s^2)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 
 


plt.subplot(2, 2, 3)
plt.plot(tall,acc3, zorder = 1)
#plt.scatter(t,acc3_sampled, c='red', zorder=2)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\ddot{\theta}_3(rad/s^2)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-4.5,4.5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-2.5,0,2.5]) 

plt.subplot(2, 2, 4)
plt.plot(tall,acc4, zorder = 1)
#plt.scatter(t,acc4_sampled, c='red', zorder=2)
plt.xlabel(r"$t(s)$")
plt.ylabel(r"$\ddot{d_4}(cm/s^2)$")
plt.xlim(0, delta_t * 11)
plt.ylim(-5, 5)
plt.xticks(np.arange(0, delta_t * 11, delta_t*3))
plt.yticks([-5,-2.5,0,2.5,5])   


plt.tight_layout()
plt.show()




#%% trajectory in Cartesian space


plt.close('all')
xx = []
yy = []
zz = []

for i in range(nump):
    theta1 = Theta1[i]
    theta2 = Theta2[i]
    theta3 = Theta3[i]
    d4 = D4[i]
    
    
    T01 = np.array([[np.cos(theta1),-np.sin(theta1),0,0], 
                    [np.sin(theta1),np.cos(theta1),0,0], 
                    [0,0,1,L1], 
                    [0,0,0,1]])

    T12 = np.array([[np.cos(theta2),-np.sin(theta2),0,L2], 
                    [0,0,1,0], 
                    [-np.sin(theta2),-np.cos(theta2),0,0], 
                    [0,0,0,1]])

    T23 = np.array([[np.cos(theta3),-np.sin(theta3),0,0], 
                    [0,0,1,L3], 
                    [-np.sin(theta3),-np.cos(theta3),0,0], 
                    [0,0,0,1]])


    T34 = np.array([[1,0,0,L4],
                    [0,1,0,0], 
                    [0,0,1,d4], 
                    [0,0,0,1]])

    #the end effector in frame{4}
    P_in4 = np.array([-L4/2,0,L4/2,1])#!!!

    #the end effector in frame{0}
    P_in0 = T01 @ T12 @ T23 @ T34 @ P_in4
    xx.append(P_in0[0])
    yy.append(P_in0[1])
    zz.append(P_in0[2])
    
    
#the actual motion
xxx = []
yyy = []
zzz = []
    
for i in range(len(comb2)):
    theta1 = comb1[i]
    theta2 = comb2[i]
    theta3 = comb3[i]
    d4 = comb4[i]
    
    
    T01 = np.array([[np.cos(theta1),-np.sin(theta1),0,0], 
                    [np.sin(theta1),np.cos(theta1),0,0], 
                    [0,0,1,L1], 
                    [0,0,0,1]])

    T12 = np.array([[np.cos(theta2),-np.sin(theta2),0,L2], 
                    [0,0,1,0], 
                    [-np.sin(theta2),-np.cos(theta2),0,0], 
                    [0,0,0,1]])

    T23 = np.array([[np.cos(theta3),-np.sin(theta3),0,0], 
                    [0,0,1,L3], 
                    [-np.sin(theta3),-np.cos(theta3),0,0], 
                    [0,0,0,1]])


    T34 = np.array([[1,0,0,L4],
                    [0,1,0,0], 
                    [0,0,1,d4], 
                    [0,0,0,1]])

    #the end effector in frame{4}
    P_in4 = np.array([-L4/2,0,L4/2,1])#!!!

    #the end effector in frame{0}
    P_in0 = T01 @ T12 @ T23 @ T34 @ P_in4
    xxx.append(P_in0[0])
    yyy.append(P_in0[1])
    zzz.append(P_in0[2])
    

    

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')


vvpoints = ax.scatter(xx,yy,zz,c='red',s = 100)
true_motion = ax.plot(xxx,yyy,zzz,color='blue', linewidth=2, linestyle='--')

ax.text(xxx[0], yyy[0], zzz[0], 'Start', color='black', fontsize=20, fontweight = 'bold')
ax.text(xxx[-1], yyy[-1], zzz[-1], 'end', color='black', fontsize=20, fontweight = 'bold')


ax.set_xlim(-50, 50)  
ax.set_ylim(-50, 50)
ax.set_zlim(0, 60)
ax.set_xlabel('X',fontsize = 15)
ax.set_ylabel('Y',fontsize = 15)
ax.set_zlabel('Z',fontsize = 15)
ax.set_title('Motion of the end effector with 10 via points',fontsize = 20)

plt.show()

