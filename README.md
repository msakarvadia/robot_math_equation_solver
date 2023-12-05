# Robot Math Equation Solver
tldr:  Use a turtlebot + arm + camera to recognize handwritten math equations and write the answer on a whiteboard.

### Project Description
In this project we enable a turtlebot to "read" a math equation using computer vision, and write the answer to that math equation on a white board (inverse kinematics). The first component of this project is computer vision: we use the turtlebot's camera to take a picture of the math equation on a white board. Then we use a convolutional neural network (which we trained) to classify the individual math digits and symbols `{0,1,2,3,4,5,6,7,8,9,+,-,*}` in the equation. We then use the python `eval()` functionality to evaluate the math equation. Once we have the answer to the math equation, we use inverse kinematics to control the motion of the robot arm and write the answer on the whiteboard. The robot arm is holding a whiteboard marker in its gripper.

### System Architecture 

All of the scripts are in the `scripts` subdir.

#### Data

- [Kaggle dataset](https://www.kaggle.com/datasets/xainano/handwrittenmathsymbols).
- Dataset was downloaded into: `scripts/data`

#### Image Pre-processing

- How to run: `python preprocess_data.py`
- This is the script that preprocesses our [Kaggle dataset](https://www.kaggle.com/datasets/xainano/handwrittenmathsymbols). It outputs the preprocessed data into `train_final.csv`.

#### Define Convolutional Neural Network (CNN)

- The class for our CNN model is defined in `nn.py`.

#### Training CNN

- How to run: `python train.py`
- The above training script saves the trained neural network checkpoint in `results/model.pth`

#### Doing inference using Robot Camera

- A test example is viewed by running `python inference.py`
- The actual script that defines how to do inference on an image is `test.py`. This file contains two critical funcitons:
    -  `equation_from_image`: Does inference on a camera image and retuns a dictionary of digits and their predicted classes
    -  `process_and_predict_answer_from_cropped_images`: Allows a human to manually correct any misclassified images, evaluates the equation, returns the answer.
- This inference is done in `robot_math_control_node.py` in the `run_inference` method of the `RobotMathControlNode` class.
- Note that prior to doing inference on an image of an equation: the image is broken up into smaller sub-images by using the `cv2` bounding box functionalty. We draw a bounding box around all continuous curves. We had to do some trouble shooting on this end to make sure we discarded bounding boxes that were too small (aka bounding boxes that had picked up a stray spec on the whiteboard).

![CV](https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/787107c2-ce09-474b-bfdd-73cdf9ce8fc7)


#### Inverse Kinematics

# TODO

### ROS Node Diagram

# TODO

### Execution

- Preprocessing data+training CNN: `python preprocess_data.py`, `python train.py`
#TODO
how to execute the rest of the live inference/IK

### Challenges

On the computer vision side we had many challenges operationalizing use of the trained NN on actual hand written whiteboard equations. This is because the equations we were writing on the board were "out of distribution" compared to the images we trained the neural network with. Whiteboards have shiny reflective surfaces: this means that there are shiny marks, shadows, leftover dry erase marker stains, random specs all around our white board. These marks would accidentally be caught by a bounding box (and then the NN would attempt to classify it). We found that we had to discard any bounding boxes that were too small to over come this challenge. Additionally, the sizing of our handdrawn charecters didn't quite match that of the training set; additinally, because our bounding boxes can have varried sizes we had to uniformly pad all of our bounding boxes and transform them to the appropriate dimentions for inference. This causes our charects to look warped (which causes misclassifications). To work around this issue, we allow a human-in-the-loop approach: we allow a humany to manual check and correct all equaitons before the are evaluated for an answer.

On the writing the answer on the board we had many challenges as well: We had to deal with the problem of the pen applying continuous pressure onto the whiteboard while writing. To overcome this we created a device that had a spring in it to put the pen in. This way we could have the gripper apply extra pressue to the pen and the pen could rely on the spring to help it maintain an appropriate pressue.

![PXL_20231201_033412801](https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/e23cd354-c6c4-4cbf-9838-b877b3658d7d)
![PXL_20231201_033212889](https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/7881fdeb-70a1-4639-a570-bca53d0fb670)


### Future Works

In future works we may attempt to train a deeper and more robust computer vision model. Right now, we only used a single dataset, and as a result, our model failed to generalize to our handwritten digits at times. To overcome this we needed to allow a human to manually correct any wrongly classified digits/symbols. Additionally, we may further tune parameters related to how our robot writes the answer on the board; right now the arm is a bit shakey and with more testing/tuning we could make this better.

### Takeaways
