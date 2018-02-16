import json
import numpy as np
import matplotlib.pyplot as plt

class LeaderBotAnalysis(object):
    def __init__(self, raw):
        self.raw = raw
        self.ctl_entries = [e['control'] for e in raw['data'] if 'control' in e.keys()]

        # Process time
        walltime_start = self.ctl_entries[0]['walltime']
        rostime_start = self.ctl_entries[0]['rostime']
        simtime_start = self.ctl_entries[0]['simtime']['secs'] + (self.ctl_entries[0]['simtime']['nsecs'] * 10**-9)
        print('Initial time difference (t_ros - t_sim) = {}'.format(rostime_start - simtime_start))

        for entry in self.ctl_entries:
            entry['walltime'] = entry['walltime'] - walltime_start
            entry['rostime'] = entry['rostime'] - rostime_start
            entry['simtime2'] = entry['simtime']['secs'] + (entry['simtime']['nsecs'] * 10**-9) - simtime_start

        # Reorganize data in to matrices
        self.ctl_steps = {
            'walltime': np.array([e['walltime'] for e in self.ctl_entries]),
            'rostime': np.array([e['rostime'] for e in self.ctl_entries]),
            'simtime': np.array([e['simtime2'] for e in self.ctl_entries]),
            'X': np.array([[e['tf_baselink_map']['x'], e['tf_baselink_map']['y'], e['tf_baselink_map']['theta']] for e in self.ctl_entries]),
            'tgt': np.array([[e['tf_target_map']['x'], e['tf_target_map']['y']] for e in self.ctl_entries]),
            'err_centerline': np.array([e['e_centerline'] for e in self.ctl_entries]),
            'err_heading': np.array([e['e_heading'] for e in self.ctl_entries]),
            'err_dist': np.array([e['e_dist'] for e in self.ctl_entries]),
            'cmd_v': np.array([e['command']['v'] for e in self.ctl_entries]),
            'cmd_w': np.array([e['command']['w'] for e in self.ctl_entries])
        }
        self.ctl_steps['t'] = self.ctl_steps['simtime']

    def __call__(self):
        # self.plot_timediffs()
        self.plot_errors()
        self.plot_motion()
        self.plot_cmds()

        plt.show()

    def plot_timediffs(self):
        plt.figure()
        plt.title('Comare ROS time and MORSE Sim time')
        plt.grid(True)
        plt.hold(True)

        plt.plot(self.ctl_steps['walltime'], 'k.')
        plt.plot(self.ctl_steps['rostime'], 'b.')
        plt.plot(self.ctl_steps['simtime'], 'r.')

        plt.legend(['Wall', 'ROS', 'SIM'])

    def plot_errors(self):
        errs = ['err_centerline', 'err_heading', 'err_dist']
        fig, axes = plt.subplots(nrows=len(errs))
        fig.suptitle('Error plots')

        for idx in range(len(errs)):
            axes[idx].plot(self.ctl_steps['t'], self.ctl_steps[errs[idx]], 'b.')
            axes[idx].grid(True)

        axes[0].set_ylabel('centerline err (m)')
        axes[1].set_ylabel('heading err (rad)')
        axes[2].set_ylabel('distance err (m)')

    def plot_motion(self):
        plt.figure()
        plt.title('Robot vs Target 2D')
        plt.grid(True)
        plt.hold(True)

        plt.plot(self.ctl_steps['X'][:, 0], self.ctl_steps['X'][:, 1], 'b.-')
        plt.plot(self.ctl_steps['tgt'][:, 0], self.ctl_steps['tgt'][:, 1], 'ro:')

        plt.ylabel('Y (m)')
        plt.xlabel('X (m)')
        plt.legend(['Robot', 'Waypoint'])

        fig, axes = plt.subplots(nrows=3)
        fig.suptitle('Robot state over time')
        for idx in range(3):
            axes[idx].plot(self.ctl_steps['t'], self.ctl_steps['X'][:, idx], 'b.')
            axes[idx].grid(True)

        axes[0].set_ylabel('X (m)')
        axes[1].set_ylabel('Y (m)')
        axes[2].set_ylabel('Theta (rad)')

    def plot_cmds(self):
        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()
        ax1.plot(self.ctl_steps['t'], self.ctl_steps['cmd_v'], 'b.')
        ax2.plot(self.ctl_steps['t'], self.ctl_steps['cmd_w'], 'g.')

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Command V (m/s)')
        ax2.set_ylabel('Command W (rad/s)')
        ax2.grid(True)

    def heading_analysis(self):
        r_T_g = np.matrix([])

if __name__ == "__main__":
    with open('leader.log') as f:
        dat = json.load(f)
        analysis = LeaderBotAnalysis(dat)
        analysis()
        
            


