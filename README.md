# multi_agent_sim

This package is my personal project for ultimately controlling multiple agents in one simulation.

Current setup is:
MORSE Simulator environment with robots and scene.
ROS middleware used to pipe world data to ROS, where SLAM/navigation/etc are carried out.

First task is to bring up a single robot to achieve SLAM and autonomous navigation. Then I'll bring in more robots for increasingly complex behaviors.

ROBOT:
A SegwayRMP4000 robot with a Hokuyo laser scanner mounted on it. Went with this robot because I would like to be able to do outdoor navigation as well.

TODOs:
Navigation stack for robot. Make my own implementation of frontier exploration (A Frontier-Based Approach for Autonomous Exploration).


