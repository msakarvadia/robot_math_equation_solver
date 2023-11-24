import torch
import cv2
from nn import Net
import numpy as np
from cv2 import Rect

network = Net()
PATH = 'results/model.pth'
network.load_state_dict(torch.load(PATH))
network.eval()

#need to use CV2 to break test.jpeg into smaller images based on 
#individal digits bounding boxes

class Point2D:
    def __init__(self, x, y):
        self.x= x
        self.y= y

class boundCorners:
    def __init__(self, x, y, w, h):
        self.topleft= Point2D(x, y)
        self.topright= Point2D(x+w, y)
        self.bottomleft= Point2D(x, y-h)
        self.bottomright= Point2D(x+w, y-h)

def preview_processing(name, img):
    #opens window to show contours and bounding rectangles to help debug
    cv2.imshow(name, img)

def equation_from_image(img):
    #get image binary. first convert image to grayscale.
    #blur image a little to help get rid of noise
    image_extract= cv2.imread(img, cv2.IMREAD_GRAYSCALE)
    image_extract= cv2.blur(image_extract, (5, 5)) #modify kernel value if needed

    #modify second variable if binarized image has too much noise from whiteboard
    #thresh_binary_inv used because analyzed equation needs to be white with black bg
    #black whiteboard marker will work best and guarantee the most success
    image_binary= cv2.threshold(image_extract, 150, 255, cv2.THRESH_BINARY_INV)

    #get contours
    #image_binary2: image with extracted contours
    #image_contours: contour data, each contour is a vector of points
    #hirearchy: hirearchy data of the contours
    #chain_approx_simple will get the endpoints of the lines of the image
    #try chain_approx_none if it doesn't work well and we need the full shape
    image_binary2, image_contours, hirearchy= cv2.findContours(image_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    preview_processing('contour preview', image_binary2)

    #get bounding boxes and coordinates of box corners
    #takes the image contours and creates bounding boxes for each contour
    #contour_curves made to approximate shapes in opencv tutorial
    ##commented out bc might not be needed
    idx= 0
    #contour_curves= []
    bounding_rectangles= []
    bounding_coords= []
    for cntr in image_contours:
        #contour_curves[idx]= cv2.approxPolyDP(cntr, 3, True)
        x, y, w, h= cv2.boundingRect(image_contours[idx])
        #(x, y) is the top left coordinate of the bounding rectagle
        bounding_rectangles[idx]= Rect(x, y, w, h)
        bounding_coords[idx]= boundCorners(x, y, w, h)
        idx += 1

    #check for overlapping rectangles
    processed_rectangles= []
    idx= 0
    """ #NOTE(MS): I just commented this out so that I can test the rest of the program"""
    #assumed bounding rectangles recognize the numbers from left to right
    #CURRENTLY PSEUDOCODE, NEED TO CHECK STRUCTURE OF BOUNDS
    for idx in range(0, bounding_rectangles.len()-1):
        #if rectangles overlap, compare sizes, send the larger one to processed_rectangle
        #skip over the next rectangle, since it was assessed here and determined to overlap
        if (bounding_coords[idx].topright.x < bounding_coords[idx+1].bottomleft.x
            or bounding_coords[idx].bottomleft.x > bounding_coords[idx+1].topright.x
            or bounding_coords[idx].topright.y < bounding_coords[idx+1].bottomleft.y
            or bounding_coords[idx].bottomleft.y > bounding_coords[idx+1].topright.y):
            #TODO: compare size of the rectangles, remove the smaller one
            idx += 2
        
        #if the two rectangles don't overlap, set processed rectangle as original
        #second rectangle needs to be checked with the one adjacent to it for overlap
        else:
            processed_rectangles[idx]= bounding_rectangles[idx]
            idx += 1

    cv2.resize(img, (28,28))
    
equation_from_image("test.jpeg")

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


    return prediction


do_inference("4_48.jpg", 4)

do_inference("0_46621.jpg", 0)

do_inference("3_40.jpg", 3)

do_inference("8_362.jpg", 8)
