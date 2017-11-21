import rospy

class AbstractNode(object):
    '''
    Abstract definition of a ROS python node
    '''
    def __init__(self, name):
        rospy.init_node(name)

    def spin(self):
        rospy.spin()

        