"""

script to define and train a neural network for image classification

"""

import pandas as pd
import numpy as np
import pickle

df_train=pd.read_csv('train_final.csv',index_col=False)

#split data into train/test
train = df_train.sample(frac=0.8)
test = df_train.drop(train.index)
print("len of test: ", len(test))
print("len of train: ", len(train))

train_labels=train[['784']]
test_labels=test[['784']]
print("unqiue labels: ", np.unique(np.array(train_labels)))

train.drop(train.columns[[784]],axis=1,inplace=True)
test.drop(test.columns[[784]],axis=1,inplace=True)

np.random.seed(1212)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

train_labels=np.array(train_labels)
test_labels=np.array(test_labels)
train_input = []
test_input = []

for i in range(len(train)):
    train_input.append(np.array(train[i:i+1], dtype=np.single).reshape(1,28,28))

for i in range(len(test)):
    test_input.append(np.array(test[i:i+1], dtype=np.single).reshape(1,28,28))

from torch.utils.data import TensorDataset, DataLoader

#train dataset/loader
tensor_x = torch.Tensor(np.array(train_input))
tensor_y = torch.squeeze(torch.tensor(train_labels))

my_dataset = TensorDataset(tensor_x,tensor_y) # create your datset
my_dataloader = DataLoader(my_dataset, shuffle=True, batch_size=64) # create your dataloader

#test dataset/loader
tensor_x = torch.Tensor(np.array(test_input))
tensor_y = torch.squeeze(torch.tensor(test_labels))

test_dataset = TensorDataset(tensor_x,tensor_y) # create your datset
test_dataloader = DataLoader(test_dataset, shuffle=True, batch_size=1000) # create your dataloader

np.random.seed(7)

### Using pytorch for model training
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

#import network from other file
from nn import Net


# hyperparameters
n_epochs = 10
learning_rate = 0.01
momentum = 0.5
log_interval = 100

random_seed = 1
torch.backends.cudnn.enabled = False
torch.manual_seed(random_seed)

network = Net()
#network(torch.from_numpy(l[0]))
#print("did successful forward pass through network")

optimizer = optim.SGD(network.parameters(), lr=learning_rate,
                      momentum=momentum)
train_losses = []
train_counter = []
test_losses = []
#test_counter = [i*len(l) for i in range(n_epochs + 1)]

# https://nextjournal.com/gkoehler/pytorch-mnist

def train(epoch):
  network.train()
  for batch_idx, (data, target) in enumerate(my_dataloader):
    optimizer.zero_grad()
    #print(data.shape)
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

def test():
  network.eval()
  test_loss = 0
  correct = 0
  with torch.no_grad():
    for data, target in test_dataloader:
      output = network(data)
      test_loss += F.nll_loss(output, target, size_average=False).item()
      pred = output.data.max(1, keepdim=True)[1]
      correct += pred.eq(target.data.view_as(pred)).sum()
  test_loss /= len(test_dataloader.dataset)
  test_losses.append(test_loss)
  print('\nTest set: Avg. loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
    test_loss, correct, len(test_dataloader.dataset),
    100. * correct / len(test_dataloader.dataset)))

test()
for epoch in range(1, n_epochs + 1):
    train(epoch)
    test()


