import torch

# Model class must be defined somewhere
PATH = 'results/model.pth'
model = torch.load(PATH)
model.eval()
