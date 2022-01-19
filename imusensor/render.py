import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import smbus
import time

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
imu.loadCalibDataFromFile("/home/pi/raspberry_pi/imusensor/calib.json")



while True:
    imu.readSensor()
    imu.computeOrientation()

    roll = imu.roll
    pitch = imu.pitch
    yaw = imu.yaw

    x1 =  (np.cos(yaw) * np.cos(roll))
    y1 = (np.cos(pitch)*np.sin(roll) + np.cos(roll) * np.sin(pitch) * np.sin(yaw))

    x2 =  (-1 * np.cos(yaw) * np.sin(roll))
    y2 =  (np.cos(pitch) * np.cos(roll) - np.sin(pitch) * np.sin(yaw) * np.sin(roll))

    x3 =  np.sin(yaw)
    y3 =  (-1 * np.cos(yaw) * np.sin(pitch))

    vectors = np.array([[0, 0 , 0, x1, y1,0], [0, 0, 0, x2, y2, 0], [0, 0, 0, x3, y3, 0]])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for vector in vectors:
        v = np.array([vector[3],vector[4],vector[5]])
        vlength=np.linalg.norm(v)
        ax.quiver(vector[0],vector[1],vector[2],vector[3],vector[4],vector[5],
           pivot='tail',length=vlength,arrow_length_ratio=0.3/vlength)

    ax.set_xlim([-4,4])
    ax.set_ylim([-4,4])
    ax.set_zlim([0,4])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    time.sleep(3)





#vectors = np.array([[0, 0 , 0, x1, y1, 0], [0, 0, 0, x2, y2, 0], [0, 0, 0, x3, y3, 0]])

#vectors=np.array( [ [0,0,1,1,-2,0], [0,0,2,1,1,0],[0,0,3,2,1,0],[0,0,4,0.5,0.7,0]]) 4
#vectors = np.array([[0,0,1, 1, 0, 0], [0, 0, 2, 0, 1, 0], [0, 0, 3, 0, 0, 1]])
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#for vector in vectors:
#    v = np.array([vector[3],vector[4],vector[5]])
#    vlength=np.linalg.norm(v)
#    ax.quiver(vector[0],vector[1],vector[2],vector[3],vector[4],vector[5],
#           pivot='tail',length=vlength,arrow_length_ratio=0.3/vlength)

#for vectors in vector:
    

