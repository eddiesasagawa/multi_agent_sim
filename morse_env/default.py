import os
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
        self.ground_truth_pose = Pose()
        self.append(self.ground_truth_pose)

        # Odometry
        self.odometry = Odometry()
        self.append(self.odometry)

        # Base laser scanner
        self.base_scan = Hokuyo()   # default --> range: 30m, field: 270deg, 1080 sample points
        self.base_scan.translate(x=0.275, z=0.4)
        self.append(self.base_scan)
        self.base_scan.properties(Visible_arc = False)
        # self.base_scan.properties(laser_range = 30.0)
        # self.base_scan.properties(resolution = 1.0)
        # self.base_scan.properties(scan_window = 180.0)
        self.base_scan.create_laser_arc()

    def add_ros_streams(self):
        self.motion.add_stream('ros', topic="/{}/cmd_vel".format(self.name))

        self.base_scan.add_stream('ros', frame_id="/laser_link_{}".format(self.name), topic="/{}/base_scan".format(self.name))
        self.ground_truth_pose.add_stream('ros', topic='/{}/pose_truth'.format(self.name))

        self.odometry.add_stream('ros', frame_id='/odom_{}'.format(self.name), child_frame_id='/base_link_{}'.format(self.name), topic="/{}/odom".format(self.name))

environments = {
    'grande_salle': {
        'path': 'laas/grande_salle',
        'robot_p0': [5.8, 3, 0.0],
        'cam_pos': [3, -10, 30],
        'cam_rot': [0.2, 0, 0.0]
    },

    'indoor-1': {
        'path': 'indoors-1/indoor-1',
        'robot_p0': [15.0, 3.2, 0.5],
        'cam_pos': [20, -5, 5],
        'cam_rot': [1.0470, 0, 0.7854]
    },

    'arena': {
        'path': '{}/arena.blend'.format(os.path.dirname(os.path.realpath(__file__))),
        'robot_p0': [5.8, 3, 0.0],
        'cam_pos': [3, -10, 30],
        'cam_rot': [0.2, 0, 0.0]
    }
}

def main():
    env_choice = environments['arena']

    r1 = SegwayRMP9000('seggy1', *(env_choice['robot_p0']))
    r1.add_ros_streams()

    # env = Environment('indoors-1/indoor-1') #Environment('land-1/buildings_2')
    env = Environment(env_choice['path'])
    env.set_camera_location(env_choice['cam_pos'])
    env.set_camera_rotation(env_choice['cam_rot'])

main()