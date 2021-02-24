# sawyer_irl_project
Author: Prasanth Sengadu Suresh.

Owned By: THINC Lab, Department of Computer Science,
          University of Georgia.

Currently Managed By: Prasanth Sengadu Suresh.

This package uses Inverse Reinforcement Learning -- Apprenticeship Learning/Learning from Demonstration to teach Sawyer Robot to perform vegetable sorting on a conveyor line alongside a Human Expert.

This package is built upon the Sawyer/Intera ROS packages and uses Robotiq 2F-85 Gripper as the End Effector.
## The following instructions are written for Ubuntu 16.04, ROS Kinetic. If you are on Melodic, check the additional changes on [Melodic migration Readme file](https://github.com/thinclab/sawyer_irl_project/blob/master/Melodic_Migration_Readme.md).

If you need to install the packages for Kinect V2 with Ubuntu, check out this [link](https://github.com/thinclab/sawyer_irl_project/blob/master/Kinect_install_readme.md).

The following are the steps to be followed to get this package working:

  Assuming you have a working version of Ubuntu (This package has been built and tested on Ubuntu 16.04)
  
  1.) Install ROS (This package was built on ROS Kinectic, for other versions you might need to make appropriate changes)
  
   [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)
      
   [Catkin Workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
      
   [Moveit Install](https://moveit.ros.org/install/)
   
   [Moveit Workspace Setup](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
   
   Also install:
          `sudo apt install ros-<YOUR ROS DISTRO>-ros-controllers`
   
  2.) We need an upgraded IK solver for smooth working of Sawyer:
  
   - Use the following command:
   
     `sudo apt-get install ros-<YOUR-ROS-DISTRO>-trac-ik-kinematics-plugin`
     
   - Then you need to find the kinematics.yaml file within sawyer_moveit_config folder and update this line: 
   `kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin` to `kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`
  
   - Here's the wiki [link](https://ros-planning.github.io/moveit_tutorials/doc/trac_ik/trac_ik_tutorial.html) for reference.
   
  2.1) This is still a work in progress, but if you want to add STOMP path planning library as a smoothing filter over OMPL, check these [instructions](https://github.com/thinclab/sawyer_irl_project/blob/master/OMPL-STOMP_smoothing_filter.md)
      
  3.) Now that you have a catkin workspace setup, in you src folder, git clone the following packages:
  
   - These packages have changes that are not a part of their default branches. Make sure you clone them from the links below.
          
          git clone --branch release-5.2.0 https://github.com/thinclab/intera_sdk.git
      
          git clone --branch release-5.2.0 https://github.com/thinclab/intera_common.git

      
   - cd into catkin_ws and do a catkin_make at this point. This will generate the intera custom messages that the following packages use.
   
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_moveit.git
      
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_robot.git
      
          git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_simulator.git
      
          git clone --branch kinetic-devel https://github.com/thinclab/robotiq.git
      
          git clone https://github.com/thinclab/roboticsgroup_gazebo_plugins-1
      
          git clone https://github.com/thinclab/gazebo_ros_link_attacher
      
          git clone https://github.com/thinclab/kinect_v2_udrf
          
          git clone https://github.com/thinclab/sawyer_irl_project.git
          
          git clone https://github.com/thinclab/velocity_plugin.git
          
          git clone https://github.com/thinclab/iai_kinect2
          
          git clone https://github.com/thinclab/sanet_onionsorting.git
          
   - Use the following command to update all your packages and drivers:
   
          sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

   - cd into catkin_ws and install all dependencies for these packages: 

          rosdep install --from-paths src --ignore-src --rosdistro=<YOUR ROS DISTRO> -y -i --verbose

     - If you see any uninstalled dependencies, you might have to manually install them using apt-get install or pip install.
     - If you still have errors, use 

           rospack depends [name of package]
 
     - This should give you the dependencies you are missing for any given package.
     - Do a catkin_make to compile. If you face a soem error, try the following command (Note: This may not be your error, check before executing this command)
     
    sudo apt-get install ros-<YOUR ROS DISTRO>-socketcan-interface ros-<YOUR ROS DISTRO>-rospy-message-converter ros-<YOUR ROS DISTRO>-effort-controllers python-pymodbus ros-<YOUR ROS DISTRO>-joystick-drivers ros-<YOUR ROS DISTRO>-soem
    
   - If the error persists, check out the below link:
                    [Soem error](https://github.com/tork-a/minas/issues/64)
     
  3.) You are almost ready to run the simulation. Double check if you have installed all the required plugins for moveit (esp moveit controllers)
  
   - Add this line to the end of your ~/.bashrc file(running it on terminal or copying it to bashrc without the 'export' part doesn't work. Copy it as it is.): 
   
   `export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/sawyer_irl_project/meshes:$GAZEBO_MODEL_PATH`
   
  4.) Run the following commands in seperate terminals:

      roslaunch sawyer_irl_project robot_gazebo_params.launch
      roslaunch sawyer_irl_project spawn_move_claim_onion.launch rviz_gui:=True
      roslaunch sawyer_irl_project pnp_node.launch
 
  5.) Now, if you want to run the same on the real Sawyer Robot,
  
   [Robot Setup](http://sdk.rethinkrobotics.com/intera/Robot_Setup)
        
   [Workstation Setup](http://sdk.rethinkrobotics.com/intera/Workstation_Setup)
        
   - Make sure you are able to connect to the gripper using the following steps: (DO ALL THESE STEPS WITHIN THE ./intera.sh ENV, ALL IN NEW TERMINALS)
        
         sudo chmod 777 /dev/ttyUSB0
          
         roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server_rtu.launch  (DRIVER FILE: Keep this running until the gripper is being used)
          
         rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 
         
   - For Sawyer, do the following steps (In new tabs, in the same order):
          
         roslaunch sawyer_irl_project upload_gripper.launch
         
         roslaunch sawyer_irl_project robot.launch
         
         roslaunch sawyer_moveit_config sawyer_moveit.launch
         
   - For obtaining coordinates using vision:
          Refer to [this link](https://github.com/thinclab/sanet_onionsorting/blob/master/README.md)
          
   - For regular pick and place:
        
         rosrun sawyer_irl_project simple_pnp.py    (Make sure all files in this folder are set to executable in file properties)
         
   - For pick and place using vision:
   
          TO BE UPDATED
