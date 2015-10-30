# Mechatronics Readme

This repository contains all the necessary code to run the mechatronics project for our team that is designed to estimate upper body joint positions using a magnetometer.

# Setting up Project

## Make Catkin Workspace 

First, create a catkin workspace (`wiki.ros.org/catkin/Tutorials/create_a_workspace`)

`mkdir -p ~/catkin_ws/src`

`cd ~/catkin_ws/src`

`catkin_init_workspace`


Then, in the directory `catkin_ws/src/`, execute

``git clone https://github.com/rjgriffin42/mechatronics``

This adds the mechatronics repo, which is where our project is  contained.

## Clone Repo

An alternative is to clone this repository, cd into scripts, and run `./install_project.sh` TODO

# Building Project

In the main directory (`${LOCATION}/catkin_ws`), execute the command `catkin_make`. This should allow you to execute the appropriate `roslaunch` commands (see `wiki.ros.org/roslaunch`)

# Contributing content

We use a feature branch system to contribute new code. Please verify that it compiles and works before issuing a pull request. As a review, to make a new branch, execute `git checkout -b $BRANCH_NAME`. To push code to the remote host, use the `git add FILES`, `git commit -m 'MESSAGE'`, and `git push origin/$BRANCH_NAME` commands. To issuing a pull request, please use the GUI on github and enter a message describing the nature of the pull request.
