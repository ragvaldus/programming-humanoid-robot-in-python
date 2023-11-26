'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''

from forward_kinematics import ForwardKinematicsAgent

import numpy as np
from scipy.linalg import pinv, norm
from scipy.spatial.transform import Rotation


class InverseKinematicsAgent(ForwardKinematicsAgent):

    def vectorize_transform(self, transform):
        rotation = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
        return [transform[3, 0], transform[3, 1], transform[3, 2], rotation[0], rotation[1], rotation[2]]

    def inverse_kinematics(self, effector_name, transform):

        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
                
        joints = self.chains[effector_name]
        num_joints = len(joints)
        angles = np.random.random(num_joints) * 1e-5
        target = self.vectorize_transform(transform)
        max_error = 0.1

        for I in range(1000):
            angle_dict = self.perception.joint
            for i, joint in enumerate(joints):
                angle_dict[joint] = angles[i]

            self.forward_kinematics(angle_dict)
            joint_transforms = [self.vectorize_transform(
                self.transforms[joint]) for joint in joints]
            end_effector_transform = np.matrix(joint_transforms[-1]).T

            error = np.matrix(target) - end_effector_transform
            error[error > max_error] = max_error
            error[error < -max_error] = -max_error

            joint_transforms_matrix = np.matrix(joint_transforms).T
            jacobian = end_effector_transform - joint_transforms_matrix
            dT = end_effector_transform - joint_transforms_matrix

            jacobian[0, :] = dT[2, :] - dT[1, :]
            jacobian[1, :] = dT[0, :] - dT[2, :]
            jacobian[2, :] = dT[1, :] - dT[0, :]
            jacobian[3:, :] = 1

            delta_angles = pinv(jacobian) * error
            angles += np.asarray(delta_angles.T)[0]

            if norm(error) < 1e-5:
                break

        return angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''

        angles_from = self.perception.joint
        angles_to_raw = self.inverse_kinematics(effector_name, transform)
        joints = self.chains[effector_name]

        angles_to = {name: angles_from[name] for name in angles_from.keys()}
        angles_to.update({joints[i]: angles_to_raw[i]
                         for i in range(len(joints))})

        names = self.chains[effector_name]
        times = [[1.0, 7.0] for _ in names]
        keys = [[[angles_from[name], [3, 0.0, 0.0], [3, 0.0, 0.0]],
                 [angles_to[name], [3, 0.0, 0.0], [3, 0.0, 0.0]]]
                for name in names]

        self.keyframes = (names, times, keys)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.eye(4)
    T[3, 1] = 0.05
    T[3, 2] = 0.26
    T[3, 0] = -0.4
    agent.set_transforms('LLeg', T)
    agent.run()
