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
- The actual script that defines how to do inference on an image is `test.py`. This file contains two critical functions:
    -  `equation_from_image`: Does inference on a camera image and returns a dictionary of digits and their predicted classes
    -  `process_and_predict_answer_from_cropped_images`: Allows a human to manually correct any misclassified images, evaluates the equation, returns the answer.
- This inference is done in `robot_math_control_node.py` in the `run_inference` method of the `RobotMathControlNode` class.
- Note that prior to doing inference on an image of an equation: the image is broken up into smaller sub-images by using the `cv2` bounding box functionality. We draw a bounding box around all continuous curves. We had to do some trouble shooting on this end to make sure we discarded bounding boxes that were too small (aka bounding boxes that had picked up a stray spec on the whiteboard).

<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/787107c2-ce09-474b-bfdd-73cdf9ce8fc7" width="90%">
</p>


#### Inverse Kinematics Base Algorithm:

The equations we used were adapted from the paper "Design of a Three Degrees of Freedom Robotic Arm" by Madiha Farman et al. As the arms on our turtle bots have 4 degrees of freedom, we limited ours to 3, for ease of calculations, by treating the 4th joint (at the base of the gripper) as fixed. The base algorithm itself is within the `node_numberwrite.py` script and in the `inv_kin()` function, which accepts 3 values for x, y, and z, and outputs the joint angles for the joint 1, 2 and 3.

#### Inverse Kinematics Planner

The inverse kinematics planner node (within the `node_numberwrite.py` script) when run receives data from the character path server, and creates a joint trajectory within the `write_num_trajectory()` function. This is done by creating a start point which is inputted into the inv_kin solver, and then subsequently doing the same for all the other points that compromise the current character. The joint trajectory is then executed, allowing the arm to fulfil the movements required to trace the character on the whiteboard, within the same function.

#### Character Paths
A character path is determined once the robot has reached a drawing position. As this project has not fully implemented writing with movement, the path should be calculated from where the robot is placed. Once the math equation has been processed by the CNN, the answer string is published on the topic `robot_math/math_strings`. This string is processed by the server from `node_character_path_planner.py`, which creates base paths for each character of the string and transforms the paths from the manipulator coordinate frame to the plane of the wall. It does this through communication with another server from `node_wall_cursor_transformation.py` which uses the `scan`, `odom`, and `robot_math/tag_position` topics to monitor robot movement and update the wall transformation matrix and y/z offsets for the next character in the string. Using servers for these two tasks allowed us to create simpler code as the inverse kinematics node can request the next character when it's ready and does not need to manage an excess amount of topics and message types. 

Essentially, the creation and distribution of character paths is controlled by the character path planner server. Strings published on `robot_math/math_string` serve as a request for the manipulator to start writing, and this server watches for these strings, creating paths as they are published and storing them in a queue.

### ROS Node Diagram
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/8fc71949-0d91-4ee9-a964-4ae52ab35ef3" width="90%">
</p>

## Execution

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
- Set LIDAR constant to "RP" / "LDS" at the top of `robot_math_utils.py`, you may also need to adjust LIDAR_OFFSET (distance along the x-axis from the lidar frame to manipulator frame)

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


## Video of Robot Solving an Equation

https://github.com/msakarvadia/robot_math_equation_solver/assets/68249601/2122ddd9-b758-4ad1-8b2f-3fa79541a3ff

And a screenshot of the terminal where the image analysis results are:

<p align="center">
    <img width="90%" alt="Screen Shot 2023-12-05 at 9 56 31 PM" src="https://github.com/msakarvadia/robot_math_equation_solver/assets/68249601/1fdc8751-ee25-4617-ae8e-7e0b1806aed5">
</p>


## Challenges

On the computer vision side we had many challenges operationalizing use of the trained NN on actual hand written whiteboard equations. This is because the equations we were writing on the board were "out of distribution" compared to the images we trained the neural network with. Whiteboards have shiny reflective surfaces: this means that there are shiny marks, shadows, leftover dry erase marker stains, random specs all around our white board. These marks would accidentally be caught by a bounding box (and then the NN would attempt to classify it). We found that we had to discard any bounding boxes that were too small to over come this challenge. Additionally, the sizing of our hand-drawn characters didn't quite match that of the training set; additionally, because our bounding boxes can have varied sizes we had to uniformly pad all of our bounding boxes and transform them to the appropriate dimensions for inference. This causes our characters to look warped (which causes misclassifications). To work around this issue, we allow a human-in-the-loop approach: we allow a human to manual check and correct all equations before the are evaluated for an answer.

When writing the solution to the math problem on the board, we had many challenges as well. As we debugged our inverse kinematics and character path planning we had to deal with the problem of the pen not applying consistent pressure on the whiteboard while writing. To overcome this, we created a device using a spring and small tube where the pen would be placed. The spring helps keep the pen stay in contact with the board without ruining it, or generating too much torque in the manipulator and shifting the robot's position.
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/e23cd354-c6c4-4cbf-9838-b877b3658d7d" width="45%">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/22324068/7881fdeb-70a1-4639-a570-bca53d0fb670" width="45%">
</p>

To help with debugging the character paths, we created a debug world in Gazebo, and several debug scripts to visualize the exact paths with respect to a wall and the lidar scans. After testing, this helped us rule out any serious issues with our character path calculations. In the below images, the red line signifies the estimated wall position where the character paths are transformed onto this plane. For some reason, the lidar points from gazebo are shifted slightly closer in Rviz, but you can match the distances through other means.

<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/d1072add-ff94-4de7-9e79-be6a242dc304" width="45%">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/4c5858f7-478d-479c-8dd5-fa303e3b58ad" width="45%">
</p>
Projection of phrase: "i love math" (pen-lifts are also traced)
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/fbc3f409-7278-46e0-b098-acaf2b5200cd" width="45%">
</p>

Even after gaining confidence in our inverse kinematics and character path planning code, the issues of maintaining consistent contact with the board still remained. We determined the root causes of our drawing inconsistencies likely come from variations related to the quality the robot's lidar and manipulator. For example, with turtlebot #1 below, the "init position" where all joint angles are zeroed is far from accurate. The weight of the arm and marker is enough for it to slouch considerably. This will have a significant effect on the end effector's position.
<p align="center">
    <img src="https://github.com/msakarvadia/robot_math_equation_solver/assets/90344922/601a684d-6874-457e-b436-01a603464d33" width="90%">
</p>

## Future Works

In future works we may attempt to train a deeper and more robust computer vision model. Right now, we only used a single dataset, and as a result, our model failed to generalize to our handwritten digits at times. To overcome this we currently need a human to manually correct any wrongly classified digits/symbols. 

Much related to developing a more robust computer vision model is the future use of tags to mark the start of a writing position on the wall. Once our computer vision allows movement of the robot we can attempt more complex tasks such as backing away from the wall, reading the math equation, and then moving to a writing position marked by a tag to write the answer. Another use-case of this capability would be the writing of long strings that require repositioning of the robot. 

Currently, our program can locate any tag in of its camera. It can also move back and forth between preset distances in front of this tagged wall. It's wall transformation service can also detect these movements and recalculate the necessary transformations for projecting characters onto the wall at the new robot positions. The only change we would need to write long strings that require repositioning, is a node that not only tracks where the tag is located, but also can determine where the last written character of the robot is.


## Takeaways

When developing applications for robots, one must always be careful that the test cases actually represent the situation in which the robot will operate. In our case, for the computer vision task the images we used to train our CNN didn't completely represent all of the possibilities that the robot will actually see. Additionally, with the inverse kinematics work we debugged a lot of our work in gazebo; however, in real life, we faced the additional challenges of making a robot hold a real pen and have it apply constant pressure. Additionally, some robot applications may be able to tolerate a certain degree of uncertainty/inaccuracy. For example, in our case since we have a human checking the outputs of our CNN during every iteration we can suffice with a smaller (less accurate network). However, in a production grade environment it may not be realistic/scalable to always have a human in the loop. In these cases, it is worthwhile spending more computational power upfront training a more powerful NN to do classifications.
