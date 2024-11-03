'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        # Used to track the time offset of the perception, time so that
        # the keyframe times actually line up
        self.timeOffs = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception, repeat=True):
        target_joints = {}
        # YOUR CODE HERE
        # Target joints should map each joint to an target angle
        names, times, keys = keyframes

        # Set the offset if not yet set or new keyframes/animation
        if self.timeOffs == 0 or (keyframes == ([], [], []) and repeat == True):
            self.timeOffs = perception.time

        # Get the current relative time
        t = perception.time - self.timeOffs

        for idx in range(len(names)):
            # check whether the current time is out of bounds for keyframes
            if t > times[idx][-1]:
                continue

            # Find inbetween which keyframes the current time sits
            lTime = 0
            rTime = 0

            for timeIDX in range(1, len(times[idx])):
                # Update the right time
                rTime = times[idx][timeIDX]

                # Out of bounds to the left or right
                if lTime > t or rTime < t:
                    # Update the left time
                    lTime = rTime

                    continue

                break

            # Calculate the incline
            incl = (lTime - t) / (lTime - rTime)

            # Calculate the Ps
            #if tIDX == 0:
            #    p0 = p1 = 0
            #else:
            p0 = keys[idx][timeIDX - 1][0]
            p1 = p0 + keys[idx][timeIDX - 1][2][2]
            p3 = keys[idx][timeIDX][0]
            p2 = p3 + keys[idx][timeIDX][1][2]

            # Set the angle
            target_joints[names[idx]] =\
                p0 * (1 - incl)**3 + p1 * 3 * incl * (1 - incl)**2 + p2 * 3 * (incl ** 2) * (1 - incl) + p3 * (incl**3)
            
        # Make sure tthe LHipYawPitch exists at all times, as it gets copied
        if target_joints.get("LHipYawPitch") == None:
            target_joints["LHipYawPitch"] = 0

        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand() # hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
