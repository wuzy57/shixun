from const import *
import rospy
from robot_state import RobotState
from rotation import *
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelConfiguration

# robot state
robot_state = RobotState()

# publish the action to corresponding topic
pub_l_ank_pitch = rospy.Publisher('/exhx5/l_ank_pitch_position/command', Float64, queue_size=10)
pub_l_ank_roll = rospy.Publisher('/exhx5/l_ank_roll_position/command', Float64, queue_size=10)
pub_l_hip_pitch = rospy.Publisher('/exhx5/l_hip_pitch_position/command', Float64, queue_size=10)
pub_l_hip_roll = rospy.Publisher('/exhx5/l_hip_roll_position/command', Float64, queue_size=10)
pub_l_hip_yaw = rospy.Publisher('/exhx5/l_hip_yaw_position/command', Float64, queue_size=10)
pub_l_knee = rospy.Publisher('/exhx5/l_knee_position/command', Float64, queue_size=10)
pub_r_ank_pitch = rospy.Publisher('/exhx5/r_ank_pitch_position/command', Float64, queue_size=10)
pub_r_ank_roll = rospy.Publisher('/exhx5/r_ank_roll_position/command', Float64, queue_size=10)
pub_r_hip_pitch = rospy.Publisher('/exhx5/r_hip_pitch_position/command', Float64, queue_size=10)
pub_r_hip_roll = rospy.Publisher('/exhx5/r_hip_roll_position/command', Float64, queue_size=10)
pub_r_hip_yaw = rospy.Publisher('/exhx5/r_hip_yaw_position/command', Float64, queue_size=10)
pub_r_knee = rospy.Publisher('/exhx5/r_knee_position/command', Float64, queue_size=10)

reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)


class AIDA(object):

    def __init__(self):
        self.rate = rospy.Rate(120)

    def reset_action(self):
        # reset robot action
        pub_l_ank_pitch.publish(0.89)
        pub_l_ank_roll.publish(-0.038)
        pub_l_hip_pitch.publish(-0.66)
        pub_l_hip_roll.publish(-0.043)
        pub_l_hip_yaw.publish(0.0)
        pub_l_knee.publish(1.5)
        pub_r_ank_pitch.publish(-0.89)
        pub_r_ank_roll.publish(0.038)
        pub_r_hip_pitch.publish(0.66)
        pub_r_hip_roll.publish(0.043)
        pub_r_hip_yaw.publish(0.0)
        pub_r_knee.publish(-1.5)
        self.rate.sleep()

    def take_action(self, action):
        # publish action
        pub_l_ank_pitch.publish(action[0])
        pub_l_ank_roll.publish(action[1])
        pub_l_hip_pitch.publish(action[2])
        pub_l_hip_roll.publish(action[3])
        pub_l_knee.publish(action[4])
        pub_r_ank_pitch.publish(action[5])
        pub_r_ank_roll.publish(action[6])
        pub_r_hip_pitch.publish(action[7])
        pub_r_hip_roll.publish(action[8])
        pub_r_knee.publish(action[9])
        pub_l_hip_yaw.publish(0.0)
        pub_r_hip_yaw.publish(0.0)
        self.rate.sleep()

    def reset(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pause()
        except (rospy.ServiceException) as e:
            print("rospause failed!'")
        rospy.wait_for_service('gazebo/reset_world')
        try:
            reset_simulation()
        except(rospy.ServiceException) as e:
            print("reset_world failed!")
        rospy.wait_for_service('gazebo/set_model_configuration')

        try:
            reset_joints("exhx5", "robot_description", ["l_ank_pitch", "l_ank_roll", "l_hip_pitch", "l_hip_roll",
                                                        "l_hip_yaw", "l_knee", "r_ank_pitch", "r_ank_roll",
                                                        "r_hip_pitch",
                                                        "r_hip_roll", "r_hip_yaw", "r_knee"],
                         [0.89, -0.038, -0.66, 0.0, 0.0, 1.5, -0.89, 0.038, 0.66, 0.0, 0.0, -1.5])
        except (rospy.ServiceException) as e:
            print("reset_joints failed!")
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        print("reset")
        robot_state.reset_state()
        robot_state.set_robot_state()
        self.reset_action()
        rospy.sleep(1.0)

    def compute_reward(self):
        effort_reward = 3.0 - 0.25 * np.sum(abs(np.asarray(robot_state.effort)))
        effort_reward = np.clip(effort_reward, -0.5, 0.5)
        # step length
        step_vector = [(robot_state.body_x - robot_state.last_body_x), (robot_state.body_y - robot_state.last_body_y)]
        step_vector = np.asarray(step_vector)
        step_len = np.linalg.norm(step_vector)
        # print ("step_len:", step_len)
        # the angle of normal vector
        cos_a, euler = cos_angle(robot_state.last_orientation, step_vector)
        if step_len < 0.001:
            robot_state.count_of_motionless += 1
        else:
            robot_state.count_of_motionless = 0
        if cos_a >= 0:
            step_reward = min(step_len * 300 * cos_a, 5.0)
        else:
            step_reward = max(step_len * 300 * cos_a, -5.0)

        # gait reward
        robot_state.gait_step += 1
        gait_reward = 0.0
        if (
                not robot_state.last_l_contact and robot_state.l_contact and robot_state.last_r_contact and robot_state.r_contact) or (
                not robot_state.last_r_contact and robot_state.r_contact and robot_state.last_l_contact and robot_state.l_contact):
            gait_vector = [robot_state.body_x - robot_state.gait_start_position[0],
                           robot_state.body_y - robot_state.gait_start_position[1]]
            cos_step, _ = cos_angle(robot_state.gait_orientation, gait_vector)
            gait_len = np.linalg.norm(gait_vector)
            gait_len_reward = np.clip(500 * gait_len * cos_step, -10.0, 15.0)
            gait_vel = 5000 * gait_len * cos_step / robot_state.gait_step
            gait_vel_reward = np.clip(gait_vel, -10.0, 15.0)
            gait_reward = gait_len_reward + gait_vel_reward
            robot_state.gait_step = 0
            robot_state.gait_orientation = robot_state.orientation
            robot_state.gait_start_position = [robot_state.body_x, robot_state.body_y]

        if not robot_state.l_contact and not robot_state.last_r_contact:
            robot_state.without_contact += 1
        else:
            robot_state.without_contact = 0
        if robot_state.without_contact >= 3:
            gait_reward = -20.0

        height_reward = 0.0
        if 0.17 < robot_state.body_height < 0.20:
            height_reward = 0.2
        else:
            height_reward = 1.5 - 100 * abs(0.185 - robot_state.body_height)
        if abs(robot_state.euler[0]) < 10:
            orientation_reward_r = 0.2
        else:
            orientation_reward_r = -min(abs(robot_state.euler[0]) / 30, 0.5)
        if abs(robot_state.euler[1]) < 10:
            orientation_reward_p = 0.2
        else:
            orientation_reward_p = -min(abs(robot_state.euler[1]) / 30, 0.5)
        orientation_reward = orientation_reward_r + orientation_reward_p
        # falling down
        fall_reward = 0.0
        if robot_state.body_height <= 0.15:
            fall_reward = -50
            robot_state.done = True
            robot_state.fall = 1
            print("Height_irregular!", robot_state.body_height)
        # elif abs(robot_state.euler[1]) >= 20 or abs(robot_state.euler[0]) >= 20:
        #     fall_reward = -50
        #     robot_state.done = True
        #     robot_state.fall = 1
        #     print("Tilt!", robot_state.euler[0], robot_state.euler[1])
        # elif abs(max(robot_state.velocity)) > 15 and abs(max(robot_state.effort)) > 4:
        #     fall_reward = -50
        #     robot_state.done = True
        #     robot_state.fall = 1
        #     print("Over torque")
        # elif robot_state.count_of_motionless > 50:
        #     robot_state.done = True
        #     fall_reward = -50
        #     print("Motionless")
        reward = effort_reward + step_reward + gait_reward + fall_reward + height_reward + orientation_reward
        print ("effort_reward", effort_reward, "step_reward:", step_reward, "gait_reward:", gait_reward, "height:",
               height_reward, "orientation:", orientation_reward)

        # update last state
        robot_state.last_body_x = robot_state.body_x
        robot_state.last_body_y = robot_state.body_y
        robot_state.last_body_height = robot_state.body_height
        robot_state.last_orientation = robot_state.orientation
        robot_state.last_l_contact = robot_state.l_contact
        robot_state.last_r_contact = robot_state.r_contact

        return reward, robot_state.done

    def state_normalization(self, state):
        # len=28, position=[0:12], orientation=[12:16], contact=[16:18], euler = [19:22], imu=[22:28]
        state = np.asarray(state)
        state[0:12] = (1.0 / pi) * state[0:12]
        state[16] = 4.7 * state[16]
        state[19:22] = (1.0 / 180) * state[19:22]
        state[22:28] = 0.1 * state[22:28]
        state = state.reshape((1, state.shape[0]))
        return state


class CallBackData(object):
    def __init__(self):
        self.init = 0

    def callbackJointStates(self, data):
        if len(data.velocity) != 0:
            # callback position
            data_position = list(data.position)
            robot_state.position = data_position
            # callback effort
            data_effort = list(data.effort)
            robot_state.effort = data_effort
            # callback effort
            data_velocity = list(data.velocity)
            robot_state.velocity = data_velocity
        else:
            robot_state.reset_state()
        robot_state.set_robot_state()

    def callback_odom(self, data):
        robot_state.orientation[0] = data.pose.pose.orientation.x
        robot_state.orientation[1] = data.pose.pose.orientation.y
        robot_state.orientation[2] = data.pose.pose.orientation.z
        robot_state.orientation[3] = data.pose.pose.orientation.w
        robot_state.body_height = data.pose.pose.position.z
        robot_state.body_x = data.pose.pose.position.x
        robot_state.body_y = data.pose.pose.position.y
        # euler
        _, euler = transform_matrix(robot_state.orientation)
        robot_state.euler = euler.tolist()
        robot_state.set_robot_state()

    def callback_r_contact(self, data):
        if len(data.states) >= 10:
            robot_state.r_contact = 1
        else:
            robot_state.r_contact = 0
        robot_state.set_robot_state()

    def callback_l_contact(self, data):
        if len(data.states) >= 10:
            robot_state.l_contact = 1
        else:
            robot_state.l_contact = 0
        robot_state.set_robot_state()

    def callback_imu(self, imu_data):
        robot_state.imu[0] = imu_data.angular_velocity.x
        robot_state.imu[1] = imu_data.angular_velocity.y
        robot_state.imu[2] = imu_data.angular_velocity.z
        robot_state.imu[3] = imu_data.linear_acceleration.x
        robot_state.imu[4] = imu_data.linear_acceleration.y
        robot_state.imu[5] = imu_data.linear_acceleration.z
        robot_state.set_robot_state()


