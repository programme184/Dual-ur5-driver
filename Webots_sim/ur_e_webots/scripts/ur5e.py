from joint import Joint


class UR5e:
    def __init__(self, joint1, joint2, joint3, joint4, joint5, joint6):
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3
        self.joint4 = joint4
        self.joint5 = joint5
        self.joint6 = joint6

    def updateJointsPositions(self, theta1, theta2, theta3, theta4, theta5, theta6):
        self.joint1.theta = theta1
        self.joint2.theta = theta2
        self.joint3.theta = theta3
        self.joint4.theta = theta4
        self.joint5.theta = theta5
        self.joint6.theta = theta6