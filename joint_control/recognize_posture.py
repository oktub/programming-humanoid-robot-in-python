'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import wipe_forehead
import pickle


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open("robot_pose.pkl", "rb")) # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        postures = ["Crouch", "Belly", "Frog", "Back", "Left", "Right", "StandInit", "Stand", "Knee", "Sit", "HeadBack"]
        joints = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch"]
        data = [perception.joint.get(joint) for joint in joints] + [perception.imu[0], perception.imu[1]]
        idx = self.posture_classifier.predict([data])[0]
        print(postures[idx])

        return postures[idx]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = wipe_forehead(None)  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
