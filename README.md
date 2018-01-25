# multi_agent_sim

This package is my personal project for ultimately controlling multiple agents in one simulation.

Current setup is:
MORSE Simulator environment with robots and scene.
ROS middleware used to pipe world data to ROS, where SLAM/navigation/etc are carried out.

First task is to bring up a single robot to achieve SLAM and autonomous navigation. Then I'll bring in more robots for increasingly complex behaviors.

ROBOT:
A SegwayRMP4000 robot with a Hokuyo laser scanner mounted on it. Went with this robot because I would like to be able to do outdoor navigation as well.

TODOs:
Navigation stack for robot. 
Migrate to using virtualenvs for isolating Morse and ROS python requirements

DEPENDENCIES:
ROS - Kinetic

Python 2.7 for ROS (ROS doesn't support Python3)
    packages:
    - numpy
    - matplotlib (not currently used; TODO: make optional)
    - seaborn (not currently used; TODO: make optional)
    - scipy (not currently used; optional for now)
    - scikit-learn (dependency for hdbscan)
    - hdbscan
    - cv2 (OpenCV2)
    

MORSE - v??
Python 3.x for MORSE
    packages:
    - just the morse installation


INSTALLATION INSTRUCTIONS:
1. Update OS

        1) sudo apt-get update
        2) sudo apt-get upgrade

2. Install pip, virtualenvwrapper

        1) sudo apt-get install python-pip
        2) sudo pip install virtualenv
        3) sudo pip install virtualenvwrapper

3. Set up virtual environments folder

        1) mkdir ~/.virtualenvs

4. Set up auto-sourcing of virtualenv

    a. edit ~/.bashrc and append following lines:

            1) export WORKON_HOME=~/.virtualenvs
            2) source /usr/local/bin/virtualenvwrapper.sh

5. Configure ROS-side environment

    a. make environment
            1) mkvirtualenv --no-site-packages mas_ros_env
            2) deactivate

    b. install ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)

    c. set up catkin workspace
            1) mkdir -p ~/catkin_ws/src
            2) catkin_make (must have sourced ROS setup)

    d. get 3rd party ROS packages:
        i) teleop-twist-keyboard:
                sudo apt-get install ros-kinetic-teleop-twist-keyboard
        2) segway_v3 (for segway_description)
                cd ~/catkin_ws/src
                git clone https://github.com/StanleyInnovation/segway_v3.git
                sudo apt-get install ros-kinetic-base-local-planner
                sudo apt-get install ros-kinetic-move-base-msgs
                cd ~/catkin_ws
                catkin_make
        
    e. install OpenCV 3
        1) make sure we're in virtual env
                workon mas_ros_env
        2) install
                pip install opencv-contrib-python

    f. pip install packages
        1. pip install numpy scipy matplotlib seaborn scikit-learn hdbscan rospkg catkin_pkg

6. Configure MORSE-side environment

    a. make environment
            mkvirtualenv -p /usr/bin/python3 --no-site-packages morse_env
            deactivate
    b. modify activation scripts for morse_env
        Since morse_env will be using python3, we need to 'disconnect' the PYTHONPATH to /opt/ros/kinetic/lib/python2.7/dist-packages

        1) add following lines to ~/.virtualenvs/morse_env/bin/postactivate
                export OLD_PYTHONPATH=$PYTHONPATH
                unset PYTHONPATH
        2) add following lines to ~/.virtualenvs/morse_env/bin/predeactivate
                export PYTHONPATH=$OLD_PYTHONPATH
                unset OLD_PYTHONPATH
        
    c. install MORSE (TODO: map to virtualenv?)
            1) sudo apt-get install morse-simulator
            2) sudo apt-get install python3-dev
            3) sudo apt-get install python3-pip
            4) sudo pip3 install rospkg catkin_pkg

7. Download this repository and clone it to ~/catkin_ws/src ...