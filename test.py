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

def view_contours_bounds():
    #opens window to show contours and bounding rectangles to help debug

def equation_from_image(img):
    #get image binary. first convert image to grayscale.
    #thresh_binary_inv used because analyzed equation needs to be white with black bg
    #blur image a little to help get rid of noise
    image_extract= cv2_imread(img, cv2.IMREAD_GRAYSCALE)
    image_extract= cv2.blue(image_extract, (5, 5)) #modify kernel value if needed
    #modify second variable if binarized image has too much noise from whiteboard
    #black whiteboard marker will work best and guarantee the most success
    image_extract= cv2.threshold(img, 150, 255, cv2.THRESH_BINARY_INV)

    #get contours and rectangular bounds
    #chain_approx_simple will get the endpoints of the lines of the image
    #try chain_approx_none if it doesn't work well
    image_contours= cv2.findContours(image_extract, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    image_bounds= cv2.boundingRect(image_extract)
    #view_contours_bounds()

    #check for overlapping rectangles

    cv2.resize(img, (28,28))

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

