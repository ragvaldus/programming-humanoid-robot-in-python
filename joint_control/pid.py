'''In this exercise you need to implement the PID controller for joints of robot.

* Task:
    1. complete the control function in PIDController with prediction
    2. adjust PID parameters for NAO in simulation

* Hints:
    1. the motor in simulation can simple modelled by angle(t) = angle(t-1) + speed * dt
    2. use self.y to buffer model prediction
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

import numpy as np
import time 
import copy
from collections import deque
from spark_agent import SparkAgent, JOINT_CMD_NAMES


class PIDController(object):
    '''a discretized PID controller, it controls an array of servos,
       e.g. input is an array and output is also an array
    '''
    def __init__(self, dt, size):
        '''
        @param dt: step time
        @param size: number of control values
        @param delay: delay in number of steps
        '''
        self.dt = dt
        self.u = np.zeros(size)
        self.e1 = np.zeros(size)
        self.e2 = np.zeros(size)
        # ADJUST PARAMETERS BELOW
        delay = 1
        self.Kp = 27
        self.Ki = 0.5
        self.Kd = 0.1
        self.y = deque(np.zeros(size), maxlen=delay + 1)

    def set_delay(self, delay):
        '''
        @param delay: delay in number of steps
        '''
        #Varför funkar inte detta? Jag fattar inte varför originalkoden inte har en deque 
        # per servo, utan bara en deque för alla servos. Detta gör att delay inte funkar
        self.y = deque(self.y, delay + 1)

    def control(self, target, sensor):
        '''apply PID control
        @param target: reference values
        @param sensor: current values from sensor
        @return control signal
        '''

        # YOUR CODE HERE

        # Since the inputs ar of type np.array we can use vectorized operations
        # to calculate the predicted value for each servo. [x,y,z] *[x,y,z] = [x^2, y^2, z^2]

        # angle(t) = angle(t-1) + speed * dt
        predicted_val = sensor + self.u * self.dt
        self.y.append(copy.deepcopy(predicted_val)) #Add to buffer a copy of predicted_val to ensure no reference is made 
        
        # # Use modelprediction
        e = target - (sensor - self.y.popleft() + predicted_val)  

        # discrete PID-function: u = Kp * e + Ki * sum(e) + Kd * (e - e_old) / dt
        #  u(tk) = u(tk−1) + (Kp + Ki∆t + Kd/∆t )e(tk) − (Kp + 2Kd/∆t )e(tk−1) + Kd/∆t e(tk−2)
        
        #(Kp + Ki*dt + Kd/dt) * e
        P = (self.Kp + self.Ki * self.dt + self.Kd / self.dt) * e
        #-(Kp + 2*Kd/dt) * e_old
        I = -(self.Kp + (2 * self.Kd) / self.dt) * self.e1
        #(Kd/dt) * e_old_old
        D = (self.Kd / self.dt) * self.e2

        #Update values for next iteration
        self.u += P + I + D
        self.e2 = self.e1
        self.e1 = e
        
        return self.u


class PIDAgent(SparkAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PIDAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.joint_names = JOINT_CMD_NAMES.keys()
        number_of_joints = len(self.joint_names)
        self.joint_controller = PIDController(dt=0.01, size=number_of_joints)
        self.target_joints = {k: 0 for k in self.joint_names}

    def think(self, perception):
        action = super(PIDAgent, self).think(perception)
        '''calculate control vector (speeds) from
        perception.joint:   current joints' positions (dict: joint_id -> position (current))
        self.target_joints: target positions (dict: joint_id -> position (target)) '''
        joint_angles = np.asarray(
            [perception.joint[joint_id]  for joint_id in JOINT_CMD_NAMES])
        target_angles = np.asarray([self.target_joints.get(joint_id, 
            perception.joint[joint_id]) for joint_id in JOINT_CMD_NAMES])
        u = self.joint_controller.control(target_angles, joint_angles)
        action.speed = dict(zip(JOINT_CMD_NAMES.keys(), u))  # dict: joint_id -> speed
        return action


if __name__ == '__main__':
    agent = PIDAgent()
    agent.target_joints['HeadYaw'] = 1.0
    agent.run()
