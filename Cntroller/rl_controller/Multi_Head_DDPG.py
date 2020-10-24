import torch
import torch.nn as nn
import torch.nn.functional as F
from const import *
from ActorNetwork import *
from CriticNetwork import *

# new
args = parser.parse_args()

class MultiHeadDDPG():
    def load_weights(self, actor_pkl, critic_pkl):
        self.actor.load_state_dict(torch.load(actor_pkl))
        self.critic.load_state_dict(torch.load(critic_pkl))

    # state_dim=84,action_dim=8,max_action=float(env.action_space.high[0])
    # from ActorNetwork:
    # self.action_scale = torch.tensor([0.4, 0.4, 0.3, 0.4, 0.4, 0.4, 0.3, 0.4]).float().to(device)
    # self.action_bias = torch.tensor([0.8, 0.0, 0.0, 1.3, -0.8, 0.0, 0.0, -1.3]).float().to(device)

    def __init__(self):
        self.actor = ActorNetwork().to(device)
        self.actor_target = ActorNetwork().to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=1e-4)

        self.critic = CriticNetwork().to(device)
        self.critic_target = CriticNetwork().to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=1e-3)
        self.replay_buffer = Replay_buffer()
        self.writer = SummaryWriter(directory)

        self.num_critic_update_iteration = 0
        self.num_actor_update_iteration = 0
        self.num_training = 0

    def select_action(self, state):
        # numpy.reshape：函数功能：给予数组一个新的形状，而不改变它的数据
        # from AIDA_HMC.py line53:s_t = state_t
        # s_t = torch.tensor(s_t.reshape((TEST_BATCH, sequence_len, state_dim)), device=device).float()
        # s_t = torch.tensor(s_t.reshape(1, 84), device=device).float()
        state = torch.FloatTensor(state.reshape(1, 84)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def update(self):
        for it in range(args.update_iteration):
            # Sample replay buffer
            # from const.py:BATCH_SIZE = 128
            batch_size=128
            x, y, u, r, d = self.replay_buffer.sample(args.batch_size)
            state = torch.FloatTensor(x).to(device)
            action = torch.FloatTensor(u).to(device)
            next_state = torch.FloatTensor(y).to(device)
            done = torch.FloatTensor(1-d).to(device)
            reward = torch.FloatTensor(r).to(device)

            # Compute the target Q value
            target_Q = self.critic_target(next_state, self.actor_target(next_state))
            target_Q = reward + (done * args.gamma * target_Q).detach()

            # Get current Q estimate
            current_Q = self.critic(state, action)

            # Compute critic loss
            critic_loss = F.mse_loss(current_Q, target_Q)
            self.writer.add_scalar('Loss/critic_loss', critic_loss, global_step=self.num_critic_update_iteration)
            # Optimize the critic
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

            # Compute actor loss
            actor_loss = -self.critic(state, self.actor(state)).mean()
            self.writer.add_scalar('Loss/actor_loss', actor_loss, global_step=self.num_actor_update_iteration)

            # Optimize the actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Update the frozen target models
            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(args.tau * param.data + (1 - args.tau) * target_param.data)

            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(args.tau * param.data + (1 - args.tau) * target_param.data)

            self.num_actor_update_iteration += 1
            self.num_critic_update_iteration += 1

    def save(self):
        torch.save(self.actor.state_dict(), directory + 'actor.pth')
        torch.save(self.critic.state_dict(), directory + 'critic.pth')
        # print("====================================")
        # print("Model has been saved...")
        # print("====================================")

    def load(self):
        self.actor.load_state_dict(torch.load(directory + 'actor.pth'))
        self.critic.load_state_dict(torch.load(directory + 'critic.pth'))
        print("====================================")
        print("model has been loaded...")
        print("====================================")




