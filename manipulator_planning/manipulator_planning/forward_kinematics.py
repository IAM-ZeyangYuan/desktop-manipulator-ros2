import numpy as np


def fk(theta1, theta2, theta3, d4, L1=0.35, L2=0.25, L3=0.02, L4=0.10):
    """
    Returns end-effector (x, y, z) given joint values.
    Adjust the math to match your DH parameters exactly.
    """
    #converting theta2%3 from urdf convention to DH convention 
    #(cuz thats what my transformation matrices uses)
    theta2 = theta2 - np.pi/2
    theta3 = theta3 + np.pi/2

    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                    [np.sin(theta1),  np.cos(theta1), 0, 0],
                    [0,               0,              1, L1],
                    [0,               0,              0, 1]])

    T12 = np.array([[ np.cos(theta2), -np.sin(theta2), 0, L2],
                    [ 0,               0,              1, 0 ],
                    [-np.sin(theta2), -np.cos(theta2), 0, 0 ],
                    [ 0,               0,              0, 1 ]])

    T23 = np.array([[ np.cos(theta3), -np.sin(theta3), 0, 0 ],
                    [ 0,               0,              1, L3],
                    [-np.sin(theta3), -np.cos(theta3), 0, 0 ],
                    [ 0,               0,              0, 1 ]])

    T34 = np.array([[1, 0, 0, L4],
                    [0, 1, 0, 0 ],
                    [0, 0, 1, d4],
                    [0, 0, 0, 1 ]])

    T4e = np.array([[1, 0, 0, -L4/2],
                    [0, 1, 0, 0    ],
                    [0, 0, 1,  L4/2],
                    [0, 0, 0, 1    ]])

    T0e = T01 @ T12 @ T23 @ T34 @ T4e
    x,y,z = T0e[:3,3]
    
    return x, y, z