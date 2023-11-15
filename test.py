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

def preview_processing(img):
    #opens window to show contours and bounding rectangles to help debug
    cv2.imshow('preview', img)

def equation_from_image(img):
    #get image binary. first convert image to grayscale.
    #thresh_binary_inv used because analyzed equation needs to be white with black bg
    #blur image a little to help get rid of noise
    image_extract= cv2.imread(img, cv2.IMREAD_GRAYSCALE)
    image_extract= cv2.blur(image_extract, (5, 5)) #modify kernel value if needed
    #modify second variable if binarized image has too much noise from whiteboard
    #black whiteboard marker will work best and guarantee the most success
    image_binary= cv2.threshold(image_extract, 150, 255, cv2.THRESH_BINARY_INV)

    #get contours and rectangular bounds
    #chain_approx_simple will get the endpoints of the lines of the image
    #try chain_approx_none if it doesn't work well
    image_binary2, image_contours= cv2.findContours(image_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #preview_processing(image_contours)

    bounding_coords= []
    i= 0
    for cntr in image_contours:
        image_bounds= cv2.boundingRect(cntr)
        bounding_coords[i]= image_bounds
        i += 1
        preview_processing(image_bounds)

    #check for overlapping rectangles
    checked_bounds= []
    i= 0
    """ #NOTE(MS): I just commented this out so that I can test the rest of the program
    #assumed bounding rectangles recognize the numbers from left to right
    #CURRENTLY PSEUDOCODE, NEED TO CHECK STRUCTURE OF BOUNDS
    for idx in range(0, bounding_coords.len()-1):
        if (bounding_coords[i].topright.x < bounding_coords[i+1].bottomleft.x
            or bounding_coords[i].bottomleft.x > bounding_coords[i+1].topright.x
            or bounding_coords[i].topright.y < bounding_coords[i+1].bottomleft.y
            or bounding_coords[i].bottomleft.y > bounding_coords[i+1].topright.y):
            checked_bounds[i]= #TODO: set bounds of this character to be combination of the two
            idx += 1
        else:
            #TODO: set bounds as the original bounds
            checked_bounds[i]=
            checked_bounds[i+1]=
            idx += 2

    cv2.resize(img, (28,28))
    """
    
#equation_from_image("test.jpeg")

def do_inference(img_name:str):
#currently we will just do inference on an image of a single digit
    #test_img = "0_46621.jpg"
    test_img = img_name
    img = cv2.imread(test_img,cv2.IMREAD_GRAYSCALE)
    img=~img
    im_resize = cv2.resize(img, (28,28))

    #print(type(im_resize))
    test_img = torch.from_numpy(im_resize.astype(np.single))
    #needs to be shape (1,1,28,28)
    test_img = test_img[None, None, :,:]
    print(test_img.shape)

    logits = network(test_img)
    print("logits: ", logits)
    #we used NLL to train network so logit w/ lowest score is prediction
    prediction = torch.argmax(logits)
    print("prediction: ", prediction)


    return prediction


do_inference("4_48.jpg")

do_inference("0_46621.jpg")
