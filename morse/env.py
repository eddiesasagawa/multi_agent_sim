from morse.builder import *

robot = BarePR2()
robot.translate(x=2.5, y=3.2, z=0.0)

odometry = Odometry()
robot.append(odometry)
odometry.add_interface('ros', topic="/odom")

keyboard = Keyboard()
robot.append(keyboard)

env = Environment('tum_kitchen/tum_kitchen')
env.set_camera_rotation([1.0470, 0, 0.7854])