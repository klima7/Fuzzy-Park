import vrep

class Tank:
    def __init__(self, ID):
        self.clientID = ID
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
    def readVelocity(self):
        velocity = (self.rightvelocity+self.leftvelocity)/2
        return velocity
