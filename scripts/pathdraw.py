import rospy

from robot_math_equation_solver.msg import PathPoint

class PathDraw:
    
    def __init__(self):
        
        rospy.init_node("robot_math_path_draw")

        self.point_pub = rospy.Publisher("/robot_math/path_draw", PathPoint, queue_size=10, latch=True),

        self.

 
    def path_callback(self, msg):












if __name__ == "__main__":
    pass