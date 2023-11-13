import torch
import cv2
from nn import Net
import numpy as np

network = Net()
PATH = 'results/model.pth'
network.load_state_dict(torch.load(PATH))
network.eval()

#need to use CV2 to break test.jpeg into smaller images based on 
#individal digits bounding boxes

#currently we will just do inference on an image of a single digit
test_img = "0_46621.jpg"
img = cv2.imread(test_img,cv2.IMREAD_GRAYSCALE)
img=~img
im_resize = cv2.resize(img, (28,28))

print(type(im_resize))
test_img = torch.from_numpy(im_resize.astype(np.single))
#needs to be shape (1,1,28,28)
test_img = test_img[None, None, :,:]
print(test_img.shape)

logits = network(test_img)
print("logits: ", logits)
#we used NLL to train network so logit w/ lowest score is prediction
prediction = torch.argmin(logits)
print("prediction: ", prediction)

