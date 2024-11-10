'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, sin, cos, array

from recognize_posture import PostureRecognitionAgent
#from ..joint_control.recognize_posture import PostureRecognitionAgent

class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
        }

        # translations from 0, 0, 0 (http://doc.aldebaran.com/2-1/family/robots/links_robot.html#robot-links)
        self.transl = {
            #                   X   Y      Z            in m
            'HeadYaw':        array([  0,   0,  126.5])/1000,
            'HeadPitch':      array([  0,   0,    0  ])/1000,
            'LShoulderPitch': array([  0,  98,  100  ])/1000,
            'LShoulderRoll':  array([  0,   0,    0  ])/1000,
            'LElbowYaw':      array([105,  15,    0  ])/1000,
            'LElbowRoll':     array([  0,   0,    0  ])/1000,
            'LHipYawPitch':   array([  0,  50, - 85  ])/1000,
            'LHipRoll':       array([  0,   0,    0  ])/1000,
            'LHipPitch':      array([  0,   0,    0  ])/1000,
            'LKneePitch':     array([  0,   0, -100  ])/1000,
            'LAnklePitch':    array([  0,   0, -102.9])/1000,
            'LAnkleRoll':     array([  0,   0,    0  ])/1000,
            'RHipYawPitch':   array([  0, -50, - 85  ])/1000,
            'RHipRoll':       array([  0,   0,    0  ])/1000,
            'RHipPitch':      array([  0,   0,    0  ])/1000,
            'RKneePitch':     array([  0,   0, -100  ])/1000,
            'RAnklePitch':    array([  0,   0, -102.9])/1000,
            'RAnkleRoll':     array([  0,   0,    0  ])/1000,
            'RShoulderPitch': array([  0, -98,  100  ])/1000,
            'RShoulderRoll':  array([  0,   0,    0  ])/1000,
            'RElbowYaw':      array([105, -15,    0  ])/1000,
            'RElbowRoll':     array([  0,   0,    0  ])/1000,
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
        # sin/cos
        js, jc = sin(joint_angle), cos(joint_angle)
        ROLL, PITCH, YAW = 0, 1, 2
        # rotations are not const, need to be put here (js and jc)
        rots = [
            # X
            array([
                [1,  0,   0, 0],
                [0, jc, -js, 0],
                [0, js,  jc, 0],
                [0,  0,   0, 1]
            ]),
            # Y
            array([
                [ jc, 0, js, 0],
                [  0, 1,  0, 0],
                [-js, 0, jc, 0],
                [  0, 0,  0, 1]
            ]),
            # Z
            array([
                [ jc, js, 0, 0],
                [-js, js, 0, 0],
                [  0,  0, 1, 0],
                [  0,  0, 0, 1]
            ])
        ]

        # select rotations
        if joint_name.endswith("Roll"):
            T = rots[ROLL]
        elif joint_name.endswith("YawPitch"):
            T = rots[YAW].dot(rots[PITCH])
        elif joint_name.endswith("Pitch"):
            T = rots[PITCH]
        elif joint_name.endswith("Yaw"):
            T = rots[YAW]
        else:
            print(f"\x1b[0;31m ERROR\x1b[0;0m Joint name \"{joint_name}\" doesn't match any class of [Roll, Pitch, Yaw]")
            print(f"\x1b[0;3em WARNING\x1b[0;0m Applying identity transform")

        # set translation
        #for idx, rv in enumerate(self.transl[joint_name]):
        #    T[idx][3] = rv[idx]
        T[0][3] = self.transl[joint_name][0]
        T[1][3] = self.transl[joint_name][1]
        T[2][3] = self.transl[joint_name][2]

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
                T = T * Tl

                self.transforms[joint] = T.copy()

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
