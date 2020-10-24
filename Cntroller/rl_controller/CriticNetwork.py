# Critic Network

import torch 
import torch.nn as nn
import torch.nn.functional as F


class CriticNetwork(nn.Module):

    def __init__(self):
        super(CriticNetwork, self).__init__()

        self.a0 = nn.Linear(8, 128)
        self.fc1 = nn.Linear(84, 128)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)

    def forward(self, state, action):

        x = F.relu(self.fc1(state))
        a = F.relu(self.a0(action))
        x = torch.cat([x, a], 1)
        x = F.relu(self.fc2(x))
        out = self.fc3(x)
        return out
