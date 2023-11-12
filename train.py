"""

script to define and train a neural network for image classification

"""

import pandas as pd
import numpy as np
import pickle

df_train=pd.read_csv('train_final.csv',index_col=False)
labels=df_train[['784']]

df_train.drop(df_train.columns[[784]],axis=1,inplace=True)
print(df_train.head())

np.random.seed(1212)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

labels=np.array(labels)
l=[]
for i in range(47504):
    l.append(np.array(df_train[i:i+1], dtype=np.single).reshape(1,28,28))

from torch.utils.data import TensorDataset, DataLoader

tensor_x = torch.Tensor(np.array(l))
tensor_y = torch.squeeze(torch.tensor(labels))
print(tensor_x.shape)
print(tensor_y.shape)
print("printed tensor datasets")

my_dataset = TensorDataset(tensor_x,tensor_y) # create your datset
my_dataloader = DataLoader(my_dataset) # create your dataloader


np.random.seed(7)

### Using pytorch for model training
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 10, kernel_size=5)
        self.conv2 = nn.Conv2d(10, 20, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.fc1 = nn.Linear(320, 50)
        self.fc2 = nn.Linear(50, 13)

    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
        x = x.view(-1, 320)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        return F.log_softmax(x)

# hyperparameters
n_epochs = 3
batch_size_train = 64
batch_size_test = 1000
learning_rate = 0.01
momentum = 0.5
log_interval = 10

random_seed = 1
torch.backends.cudnn.enabled = False
torch.manual_seed(random_seed)

network = Net()
network(torch.from_numpy(l[0]))
print("did successful forward pass through network")

optimizer = optim.SGD(network.parameters(), lr=learning_rate,
                      momentum=momentum)
train_losses = []
train_counter = []
test_losses = []
test_counter = [i*len(l) for i in range(n_epochs + 1)]

# https://nextjournal.com/gkoehler/pytorch-mnist

def train(epoch):
  network.train()
  for batch_idx, (data, target) in enumerate(my_dataloader):
    optimizer.zero_grad()
    output = network(data)
    loss = F.nll_loss(output, target)
    loss.backward()
    optimizer.step()
    if batch_idx % log_interval == 0:
      print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
        epoch, batch_idx * len(data), len(my_dataloader.dataset),
        100. * batch_idx / len(my_dataloader), loss.item()))
      train_losses.append(loss.item())
      train_counter.append(
        (batch_idx*64) + ((epoch-1)*len(my_dataloader.dataset)))
      torch.save(network.state_dict(), 'results/model.pth')
      torch.save(optimizer.state_dict(), 'results/optimizer.pth')

for epoch in range(1, n_epochs + 1):
    train(epoch)


