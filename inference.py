import torch
import cv2
from nn import Net
import numpy as np

network = Net()
PATH = 'results/model.pth'
network.load_state_dict(torch.load(PATH))
network.eval()


def do_inference(img_name:str, label:int):
    #currently we will just do inference on an image of a single digit
    #TODO: we should be able to pass in an image as well and classify it (without a file)
    #test_img = "0_46621.jpg"
    test_img = img_name
    img = cv2.imread(test_img,cv2.IMREAD_GRAYSCALE)
    img=~img
    im_resize = cv2.resize(img, (28,28))

    #print(type(im_resize))
    test_img = torch.from_numpy(im_resize.astype(np.single))
    #needs to be shape (1,1,28,28)
    test_img = test_img[None, None, :,:]
    #print(test_img.shape)

    logits = network(test_img)
    #print("logits: ", logits)
    #we used NLL to train network so logit w/ lowest score is prediction
    prediction = torch.argmax(logits)
    print("prediction: ", prediction)
    print("label: ", label)
        
    #TODO this is where we publish the prediction of the NN to a topic for the arm to write


    return prediction


do_inference("4_48.jpg", 4)

do_inference("0_46621.jpg", 0)

do_inference("3_40.jpg", 3)

do_inference("8_362.jpg", 8)
