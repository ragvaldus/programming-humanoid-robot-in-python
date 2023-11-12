'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''

from recognize_posture import PostureRecognitionAgent
from keyframes import leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        # YOUR CODE HERE

        # postures = ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack',
        #     'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']
        
        print(posture)

        if posture == "Belly":
            self.keyframes = leftBellyToStand()

        elif posture == "Back":
            self.keyframes = leftBackToStand()
        
        elif posture == "Left":
            self.keyframes = leftBackToStand()
        
        elif posture == "Right":
            self.keyframes = rightBackToStand()

        # if posture == 'Back':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'Belly':
        #     agent.keyframes = leftBellyToStand()
        # elif posture == 'Crouch':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'Frog':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'HeadBack':
        #     agent.keyframes = wipe_forehead()
        # elif posture == 'Knee':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'Left':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'Right':
        #     agent.keyframes = rightBackToStand()
        # elif posture == 'Sit':
        #     agent.keyframes = leftBackToStand()
        # elif posture == 'Stand':
        #     agent.keyframes = wipe_forehead()
        # elif posture == 'StandInit':
        #     agent.keyframes = wipe_forehead()
        # else:
        #     pass
        

class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
