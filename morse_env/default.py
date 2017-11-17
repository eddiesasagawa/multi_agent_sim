from morse.builder import *

class SegwayRMP9000(SegwayRMP400):
    def __init__(self, name, x0=0.0, y0=0.0, z0=0.0):
        SegwayRMP400.__init__(self, name)
        self.translate(x=x0, y=y0, z=z0)

        ### Actuators ###
        self.motion = MotionVWDiff()
        # self.motion.translate(z=(z0+0.3))
        self.append(self.motion)

        ### Sensors ###
        self.pose = Pose()
        self.append(self.pose)

        # Odometry
        self.odometry = Odometry()
        self.append(self.odometry)

        # Base laser scanner
        self.base_scan = Hokuyo()
        self.base_scan.translate(x=0.275, z=0.4)
        self.append(self.base_scan)
        self.base_scan.properties(Visible_arc = True)
        self.base_scan.properties(laser_range = 30.0)
        self.base_scan.properties(resolution = 1.0)
        self.base_scan.properties(scan_window = 180.0)
        self.base_scan.create_laser_arc()

    def add_ros_streams(self):
        self.motion.add_stream('ros', topic="/{}/cmd_vel".format(self.name))

        self.pose.add_stream('ros', topic="/{}/pose".format(self.name))
        self.odometry.add_stream('ros', topic="/{}/odom".format(self.name))
        self.base_scan.add_stream('ros', topic="/{}/base_scan".format(self.name))

def main():
    r1 = SegwayRMP9000('seggy1', 15.0, 3.2, 0.5)
    r1.add_ros_streams()

    # r2 = SegwayRMP9000('seggy2')
    # r2.translate(x=16.0, y=5.0, z=1.0)

    env = Environment('indoors-1/indoor-1') #Environment('land-1/buildings_2')
    env.set_camera_location([20, -5, 5])
    env.set_camera_rotation([1.0470, 0, 0.7854])

main()