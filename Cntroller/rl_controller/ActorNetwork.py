### Policy Network ###

import torch
import torch.nn as nn
import torch.nn.functional as F
from const import *


class ActorNetwork(nn.Module):
    def __init__(self):
        super(ActorNetwork, self).__init__()
        # action network

        self.fc1 = nn.Linear(84, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 8)
        self.action_scale = torch.tensor([0.4, 0.4, 0.3, 0.4, 0.4, 0.4, 0.3, 0.4]).float().to(device)
        self.action_bias = torch.tensor([0.8, 0.0, 0.0, 1.3, -0.8, 0.0, 0.0, -1.3]).float().to(device)

    def forward(self, state):

        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        x = torch.tanh(x)
        # theta action
        # dx = 0.1*torch.tanh(x)
        # action = dx + torch.cat((state[:, 0:4], state[:, 5:10], state[:, 11:12]))
        x = x * self.action_scale + self.action_bias
        return x
