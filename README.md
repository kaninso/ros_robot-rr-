# ros_robot-rr-
### software requirement:

1. Ubuntu 20.04
2. ROS (Robot Operating System) :  version 1.16.0 
3. Arduino IDE : version 1.8.15 
4. Rosserial Arduino Library 0.7.9
5. Visual Studio Code :  version 1.86.2 
6. Tkinter GUI python library

## source material:

#### How To Install Ubuntu 20.04 installation:
https://www.youtube.com/watch?v=BnV23ZEI34w&ab_channel=ProgrammingKnowledge2


#### ROS Noetic installation: 
https://www.youtube.com/watch?v=ZA7u2XPmnlo&ab_channel=RoboticsBack-End


#### Arduino installation tutorial:https:
www.youtube.com/watch?v=QTK1g0P8OUM&list=PLVZDfM16Af8nOa5SLcIAcPFzIGaJhaRgs&index=7&t=5875s&ab_channel=infinitychgg


#### Visual Studio Code 1.86.2 installation :
https://www.youtube.com/watch?v=qS-VnkhkjNI&ab_channel=ANALOGTecHII](https://www.youtube.com/watch?v=ChwsFldra-o&ab_channel=ProgrammingKnowledge)



### Hardware requirement:

1. Arduino UNO R3   1
2. Potentiometer    1
3. Encoder B1K      1
4. MG996R servo 6v  1
5. G90S servo 5v    1
6. Breadboard       1
7. USB V2.0 (Type A To Type B ) 1
8. Power Supply 5V 2A 1

### Circuit connection:

![pic ](https://github.com/Panumart22/Panumart_0104/assets/154341326/ff87c2d9-833e-4717-83b6-9cbd0bccbdb0)

### Assembly rob

![9_n](https://github.com/Panumart22/Panumart_0104/assets/154341326/3af232d0-3b59-4c0c-ba30-124986d7e15b)

### Rviz robot preview:

![429099030_388374107168104_5410754382250658877_n](https://github.com/Panumart22/Panumart_0104/assets/154341326/0427f4f8-bc03-4c25-bb15-e0adbf46dc56)


## How to Construct ROS Workspace
1. You will only need to run this command once after installing ROS for rosdep to work properly on your system.:
```bash
     sudo rosdep init
```
2. This command is the best way to set up automatic loading of ROS scripts on your system each time you open a new shell.
```bash
     echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
3. This command will immediately load the ROS script in your current shell session. This allows you to use ROS commands in that shell immediately without needing to open a new shell.
```bash
    source ~/.bashrc
```
4. Create a new directory So if you run this command from terminal
```bash
   mkdir catkin_ws
```
![a6](https://github.com/Panumart22/Panumart_0104/assets/154341326/2c1e097c-125f-430a-881e-83809080f5f0)

5. Go to the “catkin_ws” folder
```bash
    cd catkin_ws
```
6. Use the following command to create the folder name “src”
```bash
    mkdir src
```
![111](https://github.com/Panumart22/Panumart_0104/assets/154341326/5c142cac-555f-41f9-82cc-b6ac77c0167a)

7. Use the following command to construct files and folders that are the base construction in “catkin”
```bash
     catkin_make
```
8. Now, after we constructed the workspace, we are going to make every Terminal can access workspace. By using the following command, .bashrc file will be open with default text editor.
```bash
   gedit ~/.bashrc
```
10. Scroll down to the last line and press Enter. Then add the following command to .bashrc file
```bash
     source ~/catkin_ws/devel/setup.bash
```
Save the file. ROS Workspace now has been done!!

## Creat ROS Package
1. Open Terminal and go to catkin_ws directory with the following command
```bash
    cd catkin_ws
```

3. Go deeper into src folder
```bash
     cd src
```

5. At path “catkin_ws/src”, use the following command to construct the package folder name
```bash
    catkin_create_pkg (your package name) rospy
```
7. Go back to workspace directory
```bash
    cd ..
    catkin_make
```

## How to install ros_robor-rr-
1.go into src file of your ROS workspace
```bash    
    cd ~/(name your catkin workspace)/git_dir
```

2.clone github
```bash
    git clone https://github.com/kaninso/ros_robot-rr-.git
```
```bash
    cd ~/catkin_ws
```
```bash
    catkin_make
```

3.add permission to execute gui python
```bash
    cd src/git_dir/ros_robot-rr-.
```
```bash
    chmod +x MainProject.py
```
```bash
    roslaunch projectros rviz_node.launch port:="(name your port of arduino)"
```
