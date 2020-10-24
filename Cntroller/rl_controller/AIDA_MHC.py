from multi_head_DDPG import *
from const import *
import gc
gc.enable()
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from threading import Thread
from min_jerk import minimum_jerk_trajectory
import rospy
from sensor_msgs.msg import Imu
from AIDA_env import *
from AIDA_env import AIDA
from AIDA_env import robot_state

rospy.init_node('AIDA_controller')


def listener():
    print("Listener")
    call = CallBackData()
    rospy.Subscriber("/r_ank_roll_link_contact_sensor_state", ContactsState, call.callback_r_contact)
    rospy.Subscriber("/l_ank_roll_link_contact_sensor_state", ContactsState, call.callback_l_contact)
    rospy.Subscriber("/odom", Odometry, call.callback_odom)
    rospy.Subscriber("/imu", Imu, call.callback_imu)
    rospy.Subscriber("/exhx5/joint_states", JointState, call.callbackJointStates)


def publisher():
    while not rospy.is_shutdown():
        bag = np.loadtxt('./log/zmp_plan.txt')
        plan_action_bag = bag[:, [2, 8]]
        env = AIDA()
        agent = MultiHeadDDPG()
        agent.load_weights('./weights/final_weights/actor.pkl', './weights/final_weights/critic.pkl')
        plan_step_len = 6
        for episode in range(EPISODES):# 倒地状态
            print("Testing")
            reward_sum = 0.0
            # initial state_sequence
            sequence_state = np.zeros((sequence_len, state_dim))
            env.reset()
            state = robot_state.state# 获取状态
            state = env.state_normalization(state)
            # update sequence_state
            sequence_state = np.delete(sequence_state, 0, 0)
            sequence_state = np.append(sequence_state, state, axis=0)
            state_t = sequence_state


            # train
            for steps in range(1500):# 不倒地的最长步长
                s_t = state_t
                # s_t = torch.tensor(s_t.reshape((TEST_BATCH, sequence_len, state_dim)), device=device).float()
                s_t = torch.tensor(s_t.reshape(1, 84), device=device).float()
                action = agent.action(s_t)
                plan_hip_pitch = plan_action_bag[steps * plan_step_len]
                target_action = np.insert(action, 2, plan_hip_pitch[0])
                target_action = np.insert(target_action, 7, plan_hip_pitch[1])
                init_pos = np.asarray(
                    robot_state.position[0:4] + robot_state.position[5:10] + robot_state.position[11:12])
                target_pos = target_action
                # We want to move all joint from initial to target position in 1 seconds
                t, x = minimum_jerk_trajectory(init_pos, target_pos, total_time=0.06)
                for plan_i in range(1, 6):
                    plan_action = x[plan_i]
                    env.take_action(plan_action)# 执行到环境中
                reward, done = env.compute_reward()
                reward_sum += reward
                # update state
                state = robot_state.state
                state = env.state_normalization(state)
                sequence_state = np.delete(sequence_state, 0, 0)
                sequence_state = np.append(sequence_state, state, axis=0)
                state_t = sequence_state
                if done:
                    robot_state.done = False
                    print("episode", episode, "reward", reward_sum, "step_num", steps)
                    break
            print("episode", episode, "reward", reward_sum)
            robot_state.episode = episode


def main():

    # listen
    thread = Thread(target=listener(), )
    thread.start()
    # publish
    publisher()


if __name__ == '__main__':
    main()
