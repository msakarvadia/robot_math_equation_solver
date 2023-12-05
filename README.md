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

#### Inverse Kinematics

# TODO

### ROS Node Diagram

# TODO

### Execution

- Preprocessing data+training CNN: `python preprocess_data.py`, `python train.py`
- TODO: how to execute the rest of the live inference/IK

### Challenges

### Future Works

### Takeaways
