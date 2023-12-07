<<<<<<< HEAD
<<<<<<< HEAD
# robot_math_equation_solver
Use a turtlebot + arm + camera to recognize handwritten math equations and write the answer on a whiteboard


Project Description: Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate

System Architecture: Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components

ROS Node Diagram: Please include a visual diagram representing all of the ROS nodes, ROS topics, and publisher/subscriber connections present in your final project.

Execution: Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.

Challenges, Future Work, and Takeaways: These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)

=======
=======
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

<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/787107c2-ce09-474b-bfdd-73cdf9ce8fc7" width="90%">
</p>


#### Inverse Kinematics

## Inverse Kinematics Base Algorithm:

The equations we used were adapted from the paper "Design of a Three Degrees of Freedom Robotic Arm" by Madiha Farman et al. As the arms on our turtle bots have 4 degrees of freedom, we limited ours to 3, for ease of calculations, by treating the 4th joint (at the base of the gripper) as fixed. The base algorithm itself is within the `node_numberwrite.py` script and in the inv_kin() function, which accepts 3 values for x, y, and z, and outputs the joint angles for the joint 1, 2 and 3.

## Inverse Kinematics Planner

The inverse kinematcis planner node (within the `node_numberwrite.py` script) when run receives data from the character path server, and creates a joint trajectory within the write_num_trajectory() function. This is done by creating a start point which is inputted into the inv_kin solver, and then subsequently doing the same for all the other points that compromise the current character. The joint trajectory is then executed, allowing the arm to fulfil the movements required to trace the character on the whiteboard, within the same function.

## Character Path
# TODO

### ROS Node Diagram
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/8fc71949-0d91-4ee9-a964-4ae52ab35ef3" width="90%">
</p>

### Execution

Install prerequisites:
```
pip install opencv-contrib-python==4.6.0.66 scikit-image torch 
```

Preprocess training data and train CNN:
```
python preprocess_data.py && python_train.py
```

Robot preparation:
- Bring up the turtlebot with camera
- Place the turtlebot in viewing distance of where you will write math problems on the wall, also make sure it can reach the board
- Grasp a dry-erase marker with the manipulator or tape one onto it
- Set Lidar constant to "RP" / "LDS" at the top of `robot_math_utils.py`

Execute program:

```
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
```
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
```
rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw
```
```
roslaunch robot_math_equation_solver robot_math.launch 
```
-   Once the program has processed an image taken from the camera, it will display its interpretation of the math problem through a series of popups (these can be used to inspect if the handwriting is the correct size and if the camera is properly positioned)


## Video of robot performing equation solving:

https://github.com/msakarvadia/robot_math_equation_solver/assets/68249601/2122ddd9-b758-4ad1-8b2f-3fa79541a3ff

And a screenshot of the terminal where the image analysis results are:

<p align="center">
    <img width="90%" alt="Screen Shot 2023-12-05 at 9 56 31 PM" src="https://github.com/msakarvadia/robot_math_equation_solver/assets/68249601/1fdc8751-ee25-4617-ae8e-7e0b1806aed5">
</p>


### Challenges

On the computer vision side we had many challenges operationalizing use of the trained NN on actual hand written whiteboard equations. This is because the equations we were writing on the board were "out of distribution" compared to the images we trained the neural network with. Whiteboards have shiny reflective surfaces: this means that there are shiny marks, shadows, leftover dry erase marker stains, random specs all around our white board. These marks would accidentally be caught by a bounding box (and then the NN would attempt to classify it). We found that we had to discard any bounding boxes that were too small to over come this challenge. Additionally, the sizing of our handdrawn charecters didn't quite match that of the training set; additinally, because our bounding boxes can have varried sizes we had to uniformly pad all of our bounding boxes and transform them to the appropriate dimentions for inference. This causes our charects to look warped (which causes misclassifications). To work around this issue, we allow a human-in-the-loop approach: we allow a humany to manual check and correct all equaitons before the are evaluated for an answer.

When writing the solution to the math problem on the board, we had many challenges as well. As we debugged our inverse kinematics and character path planning we had to deal with the problem of the pen not applying consistent pressure on the whiteboard while writing. To overcome this, we created a device using a spring and small tube where the pen would be placed. The spring helps keep the pen in contact with the board without ruining it, or generating too much torque in the manipulator.
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/e23cd354-c6c4-4cbf-9838-b877b3658d7d" width="45%">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/7881fdeb-70a1-4639-a570-bca53d0fb670" width="45%">
</p>

After we became confident with our inverse kinematics and character path planning code and the issues of maintaining consistent contact with the board still remained, we started to look for other explanations on why the quality of our writing was so sensitive to changes in setup. To help with debugging the character coordinate paths which are passed to the inverse kinematics, we created a debug world in Gazebo, and several debug scripts to visualize the exact paths with respect to a wall and the lidar scans. After testing, this helped us rule out any serious issues with our character path calculations.
![rviz](https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/d1072add-ff94-4de7-9e79-be6a242dc304)
![gazebo](https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/4c5858f7-478d-479c-8dd5-fa303e3b58ad)
![wall_projection](https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/fbc3f409-7278-46e0-b098-acaf2b5200cd)


### Future Works

In future works we may attempt to train a deeper and more robust computer vision model. Right now, we only used a single dataset, and as a result, our model failed to generalize to our handwritten digits at times. To overcome this we needed to allow a human to manually correct any wrongly classified digits/symbols. Additionally, we may further tune parameters related to how our robot writes the answer on the board; right now the arm is a bit shakey and with more testing/tuning we could make this better.

### Takeaways

When developing applications for robots, one must always be careful that the test cases actually represent the situation in which the robot will operate. In our case, for the computer vision task the images we used to train our CNN didn't completally represent all of the possibilites that the robot will actually see. Additionally, with the inverse kinematics work we debugged a lot of our work in gazebo; however, in real life, we faced the additional challenges of making a robot hold a real pen and have it apply constant pressue. Additionally, some robot applications may be able to tolerate a certain degree of uncertainty/inaccuracy. For example, in our case since we have a human checking the outputs of our CNN during every iteration we can suffice with a smaller (less accurate network). However, in a production grade environment it may not be realistic/scalable to always have a human in the loop. In these cases, it is worthwhile spending more computational power upfront training a more powerful NN to do classifications.

=======

