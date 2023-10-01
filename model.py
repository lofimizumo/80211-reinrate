import random
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from collections import deque
from tqdm import tqdm
import random
import math
import torch.nn.init as init
from torch.distributions import Categorical


class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return np.array(state), np.array(action), np.array(reward, dtype=np.float32), np.array(next_state), np.array(done, dtype=np.uint8)

    def __len__(self):
        return len(self.buffer)

class DQNAgent(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DQNAgent, self).__init__()
        self.replay_buffer = ReplayBuffer(1000)
        self.action_dim = action_dim

        self.fc = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim)
        )
        for m in self.modules():
            if isinstance(m, nn.Linear):
                init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    m.bias.data.fill_(0.01)
        self.optimizer = optim.Adam(self.parameters(), lr=1e-3)

        self.epsilon_start = 1.0
        self.epsilon_end = 0.01
        self.epsilon_decay = 300
        self.steps_done = 0

    def get_epsilon(self):
        return self.epsilon_end + (self.epsilon_start - self.epsilon_end) * \
               math.exp(-1. * self.steps_done / self.epsilon_decay)

    def choose_action(self, state, epsilon_enable=True):
        if not epsilon_enable:
            return self(state).max(1)[1].item()
        self.steps_done += 1
        epsilon = self.get_epsilon()
        if random.random() > epsilon:
            with torch.no_grad():
                return self(state).max(1)[1].item()
        else:
            return random.choice(range(self.action_dim))

    def forward(self, state):
        #check if state is a valid tensor:
        if not torch.is_tensor(state):
            state = torch.tensor(state, dtype=torch.float32).reshape(1,-1)
        # state = (state - state.min()) / (state.max() - state.min())
        q_values = self.fc(state)
        return q_values

    def remember(self, state, action, reward, next_state, done):
        self.replay_buffer.push(state, action, reward, next_state, done)
    
    def replay(self, batch_size):
        if len(self.replay_buffer) < batch_size:
            return
        state, action, reward, next_state, done = self.replay_buffer.sample(batch_size)
        state = torch.tensor(state, dtype=torch.float32).unsqueeze(1)
        action = torch.tensor(action, dtype=torch.long).unsqueeze(1)
        reward = torch.tensor(reward, dtype=torch.float32).unsqueeze(1)
        next_state = torch.tensor(next_state, dtype=torch.float32).unsqueeze(1)
        done = torch.tensor(done, dtype=torch.float32).unsqueeze(1)

        q_values = self(state).squeeze(1)
        next_q_values = self(next_state).squeeze(1)

        action = action.type(torch.int64)
        q_value = q_values.gather(1, action).squeeze(1)

        next_q_value = next_q_values.max(1)[0]
        expected_q_value = 0.8*(reward.squeeze(1) + 0.9 * next_q_value) + 0.2*q_value 
        loss = (q_value - expected_q_value.detach()).pow(2).mean()

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

class PolicyNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(PolicyNetwork, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_size, 16),
            nn.ReLU(),
            nn.Linear(16,16),
            nn.ReLU(),
            nn.Linear(16,16),
            nn.ReLU(),
            nn.Linear(16, action_size)
        )

    def forward(self, x):
        x = self.fc(x)
        return torch.softmax(x, dim=-1)

class ReinforceAgent:
    def __init__(self, state_size, action_size, lr=1e-4):
        self.policy = PolicyNetwork(state_size, action_size)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.saved_log_probs = []
        self.rewards = []
        self.eps = 1e-8
        self.train_mode = True
        nn.init.xavier_uniform_(self.policy.fc[0].weight)
        nn.init.xavier_uniform_(self.policy.fc[2].weight)
        nn.init.xavier_uniform_(self.policy.fc[4].weight)
        nn.init.xavier_uniform_(self.policy.fc[6].weight)
    
    def eval(self):
        self.train_mode = False
        self.policy.eval()
    
    def train(self):
        self.train_mode = True
        self.policy.train()
    
    def save_model(self, model_name):
        torch.save(self.policy.state_dict(), model_name)
    
    def load_model(self, model_name):
        self.policy.load_state_dict(torch.load(model_name))

    def choose_action(self, state, max_mcs = None, epsilon=0.3):
        if not torch.is_tensor(state):
            state = torch.tensor(state, dtype=torch.float32).reshape(1,-1)
        probs = self.policy(state)
        
        if self.train_mode:
            # During training, with probability epsilon choose a random action
            if np.random.rand() < epsilon:
                action = torch.tensor([np.random.choice(len(probs[0]))])
                m = Categorical(probs)
                self.saved_log_probs.append(m.log_prob(action))
            elif max_mcs:
                action = torch.tensor(max_mcs)
                m = Categorical(probs)
                self.saved_log_probs.append(m.log_prob(action))
            else:
                m = Categorical(probs)
                action = m.sample()
                self.saved_log_probs.append(m.log_prob(action))
        else:
            # During evaluation, choose the action with the highest probability
            action = torch.argmax(probs)
            
        return action.item()

    def update(self, gamma=0.99):
        R = 0
        policy_loss = []
        returns = []
        for r in self.rewards[::-1]:
            R = r + gamma * R
            returns.insert(0, R)
        returns = torch.tensor(returns)
        if returns.std() > 0:
            returns = (returns - returns.mean()) / (returns.std() + self.eps)
        for log_prob, R in zip(self.saved_log_probs, returns):
            policy_loss.append(-log_prob * R)
        self.optimizer.zero_grad()
        policy_loss = torch.cat(policy_loss).sum()
        policy_loss.backward()
        print(f'loss: {policy_loss.item()}')
        self.optimizer.step()

        del self.rewards[:]
        del self.saved_log_probs[:]
