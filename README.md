cg_mrslam
=========

A ROS package that implements a multi-robot SLAM system.

Provides a multi-robot graph-based 2D SLAM with any assumption about data association or initial relative positions between robots.  
Handles communication among robots working in an Ad-Hoc network.

This multi-robot SLAM approach is based on the use of *condensed graphs* to share relevant map information among robots.

It is possible to use it also for single-robot SLAM.

Requirements:
-------------
- This code uses the **g2o** framework for graph optimization. *I recommend to checkout this specific **g2o** commit. I consider this a stable version, since after this commit, changes were made in the managing of Eigen and the set of some compilation flags that were giving problems in my setup*.
  
        $ git clone https://github.com/RainerKuemmerle/g2o.git
        $ cd g2o
        $ git checkout 4b9c2f5b68d14ad479457b18c5a2a0bce1541a90
  

  - Compile **g2o**:

            mkdir build && cd build
            cmake ..
            make
            cd ../g2o
            ln -s ../build/g2o/config.h .
  
  - Set up the following **g2o** environment variables in your ~/.bashrc:  

            #set up G2O
            export G2O_ROOT=path_to_your_g2o_installation  
            export G2O_BIN=${G2O_ROOT}/bin  
            export G2O_LIB=${G2O_ROOT}/lib  
            export LD_LIBRARY_PATH=${G2O_LIB}:${LD_LIBRARY_PATH}  
            export PATH=${G2O_BIN}:${PATH}  

- [ROS Fuerte](http://wiki.ros.org/fuerte/Installation), OR [ROS Indigo.](http://wiki.ros.org/indigo/Installation) OR [ROS Kinetic.](http://wiki.ros.org/kinetic/Installation)

The code has been tested on Ubuntu 12.04, 14.04 and 16.04 (64bits). 

Installation
------------
- Download the source code to your ROS workspace directory
- ROS fuerte:
  - Make sure you are in branch fuerte  

            $ git checkout fuerte
  - Type `rosmake` in the package directory
- ROS Indigo OR ROS Kinetic (catkin):
  - Installation for ROS Indigo or ROS Kinetic is supported in the default branch (master)
  - In your catkin workspace 

            $ catkin_make -DCMAKE_BUILD_TYPE=Release

Instructions
------------
There are two main programs: ```srslam```, for running a single-robot experiment and ```cg_mrslam```, for running a multi-robot experiment. ```cg_mrslam``` admits three modalities of execution (using the parameter `-modality`) for different kind of experiments:

- **real:**
  For running a multi-robot online real experiment. Robots must belong to the same network and their IP addresses configured from 192.168.0.(1.. *nRobots*) (base IP address is configurable with ```-baseAddr``` program option). 
  By default it reads /scan and /odom ROS topic although they are configurable.
  Each time a robot receives a message from the network, it publishes a message in /ping_msgs to be able to reproduce robot connectivity offline.

- **bag:**
  For reproducing bagfiles generated from a multi-robot real experiment. Reads the /ping_msgs topic to reproduce communication among robots.

- **sim:**
  For running multi-robot simulation experiments with the Stage simulator. Connectivity among robots is based on their relative distances using the ground-truth positions provided by the simulator.

The multi-robot experiment must be run using the robot namespace "robot_x", where x is the robot identifier (0.. *nRobots* -1). You can use other namespaces (e.g. myrobot_0) but the convention for robot identifiers must be maintained.
The default parameters work well in general, anyway for specific program options type:

    $ rosrun cg_mrslam mainProgram --help
  
### Example of use:

Some simulation bagfiles are provided. In the case of use them, there is no need to launch the Stage simulator.  
Open one terminal and type from the bagfiles directory (make sure you launched `roscore` before):

    $ rosbag play --pause --clock 2robots-hospital.bag

In two different terminals type:

    $ rosrun cg_mrslam cg_mrslam -idRobot 0 -nRobots 2 -scanTopic base_scan -modality sim -o testmrslam.g2o __ns:=robot_0
    $ rosrun cg_mrslam cg_mrslam -idRobot 1 -nRobots 2 -scanTopic base_scan -modality sim -o testmrslam.g2o __ns:=robot_1 

During the execution it will create one g2o file for each robot, named robot-x-testmrslam.g2o which can be openned with the **g2o_viewer**.  

Two possible visualizations can be done online using RViz. The raw graph created by the SLAM approach can be published by using the program option ```-publishGraph``` which will be drawn with respect the map frame which can be defined through the command line if it is different from the default one (/map). Additionally, it is possible to visualize the occupancy grid map published like other standard ROS mappers using the program option ```-publishMap```. For the current example, to visualize the graph and map of robot_0:

    $ rosrun cg_mrslam cg_mrslam -idRobot 0 -nRobots 2 -scanTopic base_scan -mapFrame /robot_0/map -odomFrame /robot_0/odom -modality sim -publishMap -publishGraph -o testmrslam.g2o __ns:=robot_0

Then, launch RViz (`rosrun rviz rviz`) and select /robot_0/map as the Fixed Frame in Global options. The nodes of the graph are published as a PoseArray message on the /robot_0/trajectory topic while their associated laserscans are published as a PointCloud on the /robot_0/lasermap topic. See the following screenshots as example of visualization:

![cg_mrslam](/bagfiles/cg_mrslam.png)

![cg_mrslam_gridmap](/bagfiles/cg_mrslam_gridmap.png)

Another possibility is to convert the .g2o file into an occupancy grid compatible with ROS/Stage using the following tool:
https://bitbucket.org/mtlazaro/g2o2ros

Related papers
---------------
Please, cite the following paper when using this code:  

- M.T. Lazaro, L.M. Paz, P.Pinies, J.A. Castellanos and G. Grisetti. Multi-Robot SLAM using Condensed Measurements. IEEE/RSJ International Conference on Intelligent Robots and Systems, Tokyo Big Sight, Japan, Nov 3-8, pp. 1069-1076, 2013.

Acknowledgements
----------------
- Giorgio Grisetti for his guidance in the development of this project during my stay at La Sapienza University of Rome.
- Taigo M. Bonnani from La Sapienza University of Rome for his collaboration in the implementation of the scan matcher.

