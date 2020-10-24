# 当前机器人的状态
import numpy


class RobotState(object):
    def __init__(self):

        # state.position
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # state.velocity
        self.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # state.effort
        self.effort = [-0.02, -0.496, -0.01, 0.471, 0.018, 0.0,  0.213, 0.101, -0.173, -0.08, -0.014, -0.185]
        # state.robot_orientation
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.last_orientation = [0.0, 0.0, 0.0, 1.0]
        self.euler = [0.0, 0.0, 0.0]
        # state.imu
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # state.robot_position
        self.body_height = 0.213649069412
        self.body_x = 0.0
        self.body_y = 0.0
        self.last_body_height = 0.213649069412
        self.last_body_x = 0.0
        self.last_body_y = 0.0
        # state.contact
        self.r_contact = 1
        self.l_contact = 1
        self.last_r_contact = 1
        self.last_l_contact = 1
        self.r_wheel_contact = 0
        self.l_wheel_contact = 0
        self.body_contact = 0
        self.gait_orientation = [0.0, 0.0, 0.0, 1.0]
        self.gait_step = 0
        self.gait_start_position = [0.0, 0.0]
        self.support_mode = 1
        self.last_support_mode = 1

        self.contact_height = [self.body_height, self.l_contact, self.r_contact]
        # state.policy_input
        self.state = self.position + self.orientation + self.contact_height + self.euler + self.imu

        # reward
        self.latest_reward = 0.0
        self.best_reward = -10000.0
        self.worst_reward = 1000
        self.episode = 0
        self.avg_reward = 0.0

        # other
        self.last_time = 0.0
        self.fall = 0
        self.done = False
        self.count_of_motionless = 0

    def set_robot_state(self):
        # set state
        self.contact_height = [self.body_height, self.l_contact, self.r_contact]
        # len=43, position=[0:12], velocity=[12: 24], effort=[24:36], orientation=[36:40],
        # contact=[40:43], euler = [43:46], imu=[46:52]
        # len=28, position=[0:12], orientation=[12:16], contact=[17:19], euler = [19:22], imu=[22:28]
        self.state = self.position + self.orientation + self.contact_height + self.euler + self.imu

    def reset_state(self):
        # state.position
        self.position = [0.89, -0.038, -0.66, -0.043, 0.0, 1.5, -0.89, 0.038, 0.66, 0.043, 0.0, -1.5]
        # state.robot_orientation
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.last_orientation = [0.0, 0.0, 0.0, 1.0]
        self.euler = [0.0, 0.0, 0.0]
        # state.imu
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # state.robot_position
        self.body_height = 0.213649069412
        self.body_x = 0.0
        self.body_y = 0.0
        self.last_body_x = 0.0
        self.last_body_y = 0.0
        self.last_body_height = 0.213649069412
        # state.contact
        self.r_contact = 1
        self.l_contact = 1
        self.last_r_contact = 1
        self.last_l_contact = 1
        self.r_wheel_contact = 0
        self.l_wheel_contact = 0
        self.body_contact = 0
        self.count_of_motionless = 0
        self.gait_orientation = [0.0, 0.0, 0.0, 1.0]
        self.gait_step = 0
        self.gait_start_position = [0.0, 0.0]
