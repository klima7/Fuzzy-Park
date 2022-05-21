import sys
import math
from api import vrep
import numpy as np
import matplotlib.pyplot as plt


class Distances:

    def __init__(self, en, en2, es, es2, ne, ne2, nw, nw2, se, se2, sw, sw2, wn, wn2, ws, ws2):
        self.en = min(en, 6)
        self.es = min(es, 6)
        self.ne = min(ne, 6)
        self.nw = min(nw, 6)
        self.se = min(se, 6)
        self.sw = min(sw, 6)
        self.wn = min(wn, 6)
        self.ws = min(ws, 6)

        self.en2 = min(en2, 6)
        self.es2 = min(es2, 6)
        self.ne2 = min(ne2, 6)
        self.nw2 = min(nw2, 6)
        self.se2 = min(se2, 6)
        self.sw2 = min(sw2, 6)
        self.wn2 = min(wn2, 6)
        self.ws2 = min(ws2, 6)

    def __repr__(self):
        return f'NW:{self.nw:.2f} NE:{self.ne:.2f} WN:{self.wn:.2f} EN:{self.en:.2f} | SW:{self.sw:.2f} SE:{self.se:.2f} WS:{self.ws:.2f} ES:{self.es:.2f}'


class Tank:
    def __init__(self):
        self.clientID = self.connect()
        # get handles to robot drivers
        err_code, self.left_front_handle =  vrep.simxGetObjectHandle(self.clientID,'left_front', vrep.simx_opmode_blocking)
        err_code, self.left_back_handle  =  vrep.simxGetObjectHandle(self.clientID,'left_back', vrep.simx_opmode_blocking)
        err_code, self.right_back_handle =  vrep.simxGetObjectHandle(self.clientID,'right_back', vrep.simx_opmode_blocking)
        err_code, self.right_front_handle=  vrep.simxGetObjectHandle(self.clientID,'right_front', vrep.simx_opmode_blocking)
        
        self.side_handles=[]
        for l in 'rl':
            for i in range(1,7):
                err_code, handle=  vrep.simxGetObjectHandle(self.clientID,'sj_'+l+'_'+str(i) , vrep.simx_opmode_blocking)
                self.side_handles.append(handle)
       
        #initial velocity
        self.leftvelocity=0
        self.rightvelocity=0
        self.MaxVel=10
        self.dVel=1

        # proximity
        self.proximity_sensors = ["EN", "ES", "NE", "NW", "SE", "SW", "WN", "WS"]
        self.proximity_sensors_handles = [0] * 8

        # get handle to proximity sensors
        for i in range(len(self.proximity_sensors)):
            err_code, self.proximity_sensors_handles[i] = vrep.simxGetObjectHandle(
                self.clientID, "Proximity_sensor_" + self.proximity_sensors[i], vrep.simx_opmode_blocking)

        # proximity sensors initialization
        for sensor_name, sensor_handle in zip(self.proximity_sensors, self.proximity_sensors_handles):
            err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                self.clientID, sensor_handle, vrep.simx_opmode_streaming)

        self.distances_history = []

    def connect(self):
        vrep.simxFinish(-1)
        client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if client_id != -1:
            print("Connected to remote API server")
        else:
            print("Not connected to remote API server")
            sys.exit("Could not connect")

        return client_id

    def stop(self):
        #set divers to stop mode
        force =0
        err_code = vrep.simxSetJointForce(self.clientID, self.left_front_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.left_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_front_handle, force, vrep.simx_opmode_oneshot)
        
        force =10
        for h in self.side_handles:
            err_code = vrep.simxSetJointForce(self.clientID, h, force, vrep.simx_opmode_oneshot)
        
        #break
        self.leftvelocity=10
        self.rightvelocity=10
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_front_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_back_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_back_handle,self.rightvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_front_handle,self.rightvelocity,vrep.simx_opmode_streaming)
    
    def go(self):
        #set divers to go mode
        force =10
        err_code = vrep.simxSetJointForce(self.clientID, self.left_front_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.left_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_back_handle, force, vrep.simx_opmode_oneshot)
        err_code = vrep.simxSetJointForce(self.clientID, self.right_front_handle, force, vrep.simx_opmode_oneshot)
        
        force =0
        for h in self.side_handles:
            err_code = vrep.simxSetJointForce(self.clientID, h, force, vrep.simx_opmode_oneshot)
    
    def setVelocity(self):
        #verify if the velocity is in correct range
        if self.leftvelocity > self.MaxVel:
            self.leftvelocity = self.MaxVel
        if self.leftvelocity < -self.MaxVel:
            self.leftvelocity = -self.MaxVel
        if self.rightvelocity > self.MaxVel:
            self.rightvelocity = self.MaxVel
        if self.rightvelocity < -self.MaxVel:
            self.rightvelocity = -self.MaxVel
        
        #send value of velocity to drivers
        #vrep.simxSetJointTargetVelocity(clientID,left_front_handle,leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.left_back_handle,self.leftvelocity,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID,self.right_back_handle,self.rightvelocity,vrep.simx_opmode_streaming)
        #vrep.simxSetJointTargetVelocity(clientID,right_front_handle,rightvelocity,vrep.simx_opmode_streaming)
    
    #Move the tank forward
    #None - increases velocity by 1, if velocities of wheels are different they are equalized 
    #velocity - takes values from <-10,10> and sets them as velocity for both wheels in forward direction
    def forward(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity=velocity
            self.rightvelocity=velocity
        else:
            self.rightvelocity=self.leftvelocity=(self.leftvelocity+self.rightvelocity)/2
            self.leftvelocity+=self.dVel
            self.rightvelocity+=self.dVel
        self.setVelocity()
    
    #Move the tank backward 
    #None - decreases velocity by 1, if velocities of wheels are different they are equalized 
    #velocity - takes values from <-10,10> and sets them as velocity for both wheels in backward direction
    def backward(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity=-velocity
            self.rightvelocity=-velocity
        else:
            self.rightvelocity=self.leftvelocity=(self.leftvelocity+self.rightvelocity)/2
            self.leftvelocity-=self.dVel
            self.rightvelocity-=self.dVel
        self.setVelocity()
    
    #Turns left the tank 
    #None - increases velocity of rightwheel by 1, decreases velocity of leftwheel by 1 
    #velocity - takes values from <-10,10> and sets it as velocity for right wheel 
        #in forward direction and oposite value of velocity for left wheel in backward direction
    def turn_left(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity =-velocity
            self.rightvelocity= velocity
        else:
            self.leftvelocity -=self.dVel
            self.rightvelocity+=self.dVel
        self.setVelocity()

    def turn_left_circle(self, velocity):
        self.go()
        self.leftvelocity = 10 * (velocity / 10)
        self.rightvelocity = 3 * (velocity / 10)
        self.setVelocity()


    def turn_right_circle(self, velocity):
        self.go()
        self.rightvelocity = 10 * (velocity / 10)
        self.leftvelocity = 3 * (velocity / 10)
        self.setVelocity()
    
    #Turns right the tank 
    #None - increases velocity of leftwheel by 1, decreases velocity of rightwheel by 1 
    #velocity - takes values from <-10,10> and sets it as velocity for left wheel 
        #in forward direction and oposite value of velocity for right wheel in backward direction
    def turn_right(self, velocity=None):
        self.go()
        if velocity!=None:
            self.leftvelocity = velocity
            self.rightvelocity=-velocity
        else:
            self.leftvelocity +=self.dVel
            self.rightvelocity-=self.dVel
        self.setVelocity()

    def read_distances(self):
        distances = []
        for sensor_name, sensor_handle in zip(self.proximity_sensors, self.proximity_sensors_handles):
            err_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self.clientID, sensor_handle, vrep.simx_opmode_buffer)
            norm = np.linalg.norm(detectedPoint)
            distance = norm if err_code == 0 and norm > 1e-2 else math.inf
            distance2 = distance if detectionState else math.inf
            distances.extend([distance, distance2])

        distances = Distances(*distances)
        self.distances_history.append(distances)
        return distances

    def restart_plot(self):
        self.distances_history.clear()

    def plot_distances(self):
        x = np.arange(len(self.distances_history)) / 10

        nw = [d.nw for d in self.distances_history]
        ne = [d.ne for d in self.distances_history]
        wn = [d.wn for d in self.distances_history]
        en = [d.en for d in self.distances_history]
        sw = [d.sw for d in self.distances_history]
        se = [d.se for d in self.distances_history]
        ws = [d.ws for d in self.distances_history]
        es = [d.es for d in self.distances_history]

        plt.plot(x, nw, '-', label='nw')
        plt.plot(x, ne, '-', label='ne')
        plt.plot(x, wn, '-', label='wn')
        plt.plot(x, en, '-', label='en')
        plt.plot(x, sw, '--', label='sw')
        plt.plot(x, se, '--', label='se')
        plt.plot(x, ws, '--', label='ws')
        plt.plot(x, es, '--', label='es')

        plt.legend()
        plt.show()
