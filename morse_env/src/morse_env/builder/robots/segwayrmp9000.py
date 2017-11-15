from morse.builder import *

class Segwayrmp9000(GroundRobot):
    """
    A template robot model for segwayrmp9000, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # segwayrmp9000.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'morse_env/robots/segwayrmp9000.blend', name)
        self.properties(classpath = "morse_env.robots.segwayrmp9000.Segwayrmp9000")

        ###################################
        # Actuators
        ###################################


        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.append(self.motion)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)

