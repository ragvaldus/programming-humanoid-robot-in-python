# '''In this exercise you need to implement forward kinematics for NAO robot

# * Tasks:
#     1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
#        The documentation from Aldebaran is here:
#        http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
#     2. implement the calculation of local transformation for one joint in function
#        ForwardKinematicsAgent.local_trans. The necessary documentation are:
#        http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
#        http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
#     3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
#        coordinate into self.transforms of class ForwardKinematicsAgent

# * Hints:
#     1. the local_trans has to consider different joint axes and link parameters for different joints
#     2. Please use radians and meters as unit.
# '''

# # add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from numpy import cos, sin, dot


from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = { 'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       
                        'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                        'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],

                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       }
        
        # joint_offset defines the offset of joint in the chain
        
        self.joint_offset = {'HeadYaw': [0.00, 0.00, 126.50], 
                             'HeadPitch': [0.00, 0.00, 0.00],
                            # Left arm
                             'LShoulderPitch': [0.00, 98.00, 100.00], 
                             'LShoulderRoll': [0.00, 0.00, 0.00],
                             'LElbowYaw': [105.00, 15.00, 0.00], 
                             'LElbowRoll': [0.00, 0.00, 0.00],
                            # Right arm
                             'RShoulderPitch': [0.00, -98.00, 100.00], 
                             'RShoulderRoll': [0.00, 0.00, 0.00],
                             'RElbowYaw': [105.00, -15.00, 0.00], 
                             'RElbowRoll': [0.00, 0.00, 0.00],
                            # Left leg
                             'LHipYawPitch': [0.00, 50.00, -85.00], 
                             'LHipRoll': [0.00, 0.00, 0.00],
                             'LHipPitch': [0.00, 0.00, 0.00], 
                             'LKneePitch': [0.00, 0.00, -100.00],
                             'LAnklePitch': [0.00, 0.00, -102.90], 
                             'LAnkleRoll': [0.00, 0.00, 0.00],
                            # Right leg
                             'RHipYawPitch': [0.00, -50.00, -85.00], 
                             'RHipRoll': [0.00, 0.00, 0.00],
                             'RHipPitch': [0.00, 0.00, 0.00],
                             'RKneePitch': [0.00, 0.00, -100.00], 
                             'RAnklePitch': [0.00, 0.00, -102.90],
                             'RAnkleRoll': [0.00, 0.00, 0.00]
                             }
        

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        c = cos(joint_angle)
        s = sin(joint_angle)

        # If jointname ends with roll ie x axis rotation
        # Transform to a 4x4 matrix for all
        if joint_name.endswith('Roll'):
            T = matrix([[1, 0, 0, 0],
                        [0, c,-s, 0],
                        [0, s, c, 0],
                        [0, 0, 0, 1]])
            
        # If jointname ends with pitch ie y axis rotation
        elif joint_name.endswith('Pitch'):
            T = matrix([[ c, 0, s, 0],
                        [ 0, 1, 0, 0],
                        [-s, 0, c, 0],
                        [ 0, 0, 0, 1]])
            
        # If jointname ends with yaw ie z axis rotation
        elif joint_name.endswith('Yaw'):
            T = matrix([[c,-s, 0, 0],
                        [s, c, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        
        # The joint offset is added to the transformation matrix according to 3D coordinate transformation
        T[0:3, 3] = self.joint_offset[joint_name][0]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                # Multiply all the local transformations to get the global transformation

                T = dot(T, Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()