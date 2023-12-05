# Robot Math Equation Solver
tldr:  Use a turtlebot + arm + camera to recognize handwritten math equations and write the answer on a whiteboard.

### Project Description
In this project we enable a turtlebot to "read" a math equation using computer vision, and write the answer to that math equation on a white board (inverse kinematics). The first component of this project is computer vision: we use the turtlebot's camera to take a picture of the math equation on a white board. Then we use a convolutional neural network (which we trained) to classify the individual math digits and symbols `{0,1,2,3,4,5,6,7,8,9,+,-,*}` in the equation. We then use the python `eval()` functionality to evaluate the math equation. Once we have the answer to the math equation, we use inverse kinematics to control the motion of the robot arm and write the answer on the whiteboard. The robot arm is holding a whiteboard marker in its gripper.

### System Architecture

#### Image Pre-processing

#### Training CNN

#### Inverse Kinematics

### ROS Node Diagram

### Execution

### Challenges

### Future Works

### Takeaways
