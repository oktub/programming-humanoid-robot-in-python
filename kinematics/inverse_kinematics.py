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
from numpy.matlib import *

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def from_trans(self, m):
        angle_x, angle_y, angle_z = 0, 0, 0
        if m[0, 0] == 1:
            angle_x = arctan2(m[2, 1], m[1, 1])
        elif m[1, 1] == 1:
            angle_y = arctan2(m[0, 2], m[0, 0])
        elif m[2, 2] == 1:
            angle_z = arctan2(m[1, 0], m[0, 0])

        return [m[0, -1], m[1, -1], m[2, -1], angle_x, angle_y, angle_z]

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = [0] * len(self.chains[effector_name])
        # YOUR CODE HERE
        target = array(self.from_trans(transform)).T

        lambda_ = 1
        max_step = 0.07
        for i in range(5000):
            # forward_kinematics will only use relevant joints
            self.forward_kinematics(self.perception.joint)
            
            # get matrices as a list
            Ts = list(map(lambda name: self.transforms[name], self.chains[effector_name]))
            Te = matrix([self.from_trans(Ts[-1])])
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = list(map(lambda m: self.from_trans(m), Ts))
            J = Te - T
            J[:, -1] = 1
            d_theta = lambda_ * dot(dot(J, linalg.pinv(dot(J.T, J))), e.T)

            for i in range(len(self.chains[effector_name])):
                joint_angles[i] += asarray(d_theta.T)[0][i]

            if linalg.norm(d_theta) < 1e-4:
                break

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in

        angles = self.inverse_kinematics(effector_name, transform)
        chain = self.chains[effector_name]
        times = [[0, 2]] * len(chain)
        keys = [[
            [self.perception.joint[joint], [3, -1, 0], [3, 1, 0]],
            [angles[i], [3, -1, 0], [3, 1, 0]]
        ] for i, joint in enumerate(chain)]
        self.keyframes = (chain, times, keys)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
