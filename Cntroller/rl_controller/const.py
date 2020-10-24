import torch

# Hyper Parameters:
EPISODES = 100000
noise_episode = 20000
REPLAY_BUFFER_SIZE = 300000
REPLAY_START_SIZE = 20000
sequence_len = 3
BATCH_SIZE = 128
state_dim = 28
action_dim = 8
reward_dim = 6
GAMMA = 0.99
LRA = 0.00001
LRC = 0.0001
tau = 0.01
seed = 56
pi = 3.1415926
position = (1.0, 0.0, 0.0)
USE_CUDA = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
