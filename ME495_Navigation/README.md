# HW4

To run this package, first do: 
```
mkdir /hw4
$cd /hw4/src
$git clone git@github.com:ME495-EmbeddedSystems/homework-4-RicoJia.git
$cd /hw4
$catkin_make
$source devel/setup.bash
```

## Part 1 Image Processing 
In this part, a ros node is created with an object detection feature using webcam. The webcam is started using usbcam package, and object detection is achieved by using [YOLO ROS](https://github.com/leggedrobotics/darknet_ros). 

Downloading and compiling this package may take a while! Do something else while it compiles.

![alt text](https://user-images.githubusercontent.com/39393023/69156072-7ca20d80-0aa8-11ea-9b87-e6163735c16d.png)

To run the image processing node, do: 

```
$ roslaunch hw4 HW4_image_processing.launch
```

## Part 2 TURTLEBOT3 Navigation and Mapping
### Question 1 and 2
In these two questions, you will see a turtlebot3 is loaded in the house environment, rviz and laser scan is loaded. Teleoperation node and gmapping are also started automatically. Then you can navigate the turtlebot3 using the teleop node. 

![Screenshot from 2019-11-19 09-03-18](https://user-images.githubusercontent.com/39393023/69158166-9bee6a00-0aab-11ea-9c81-7905d5dfe678.png)
![Screenshot from 2019-11-19 09-03-36](https://user-images.githubusercontent.com/39393023/69158174-9ee95a80-0aab-11ea-9a56-d5113224431d.png)

In the same shell session, do 
```
$ export TURTLEBOT3_MODEL= burger
$ roslaunch hw4 task2.launch
```

For question 2, you can see the full map of the house in the map directory. 
- How to save a map with ROS map server:

```
$cd hw4/src/maps
$rosrun map_server map_saver -f hw4_map
```

## Question 3 and 4
### Question 3
This launchfile will launch move_base, amcl, and the map previously saved from question 2. It also passes a good initial estimate to amcl
so the amcl can have a more accurate estimate of the robot's location in a faster manner. The good estimate was achieved by checking /gazebo/model_state.
To see the customized map with the motion plan, the static map, the costmap, and the costs for the global plan should be visible, run
```
  $ roslaunch hw4 task4.launch
```

To make the plannar fail, go to launch/move_base_hw4.launch, and disable the parameters for planning by commenting out
```
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
    <rosparam file="$(find turtlebot3_navigation)/param/dwa_local_planner_params_$(arg model).yaml" command="load" />
```

  Alternatively, we can select the goal to be an unreachable location, for example inside an obstacle. Then the robot will be stuck at the obstacle and cannot move. The plannar will fail accordingly

## Question 4 Adding Obstacles to Map
1. Two new obstacles are added to the house. One is immediately observable in the robot's range of view, the other is not.

![Screenshot from 2019-11-19 09-14-04](https://user-images.githubusercontent.com/39393023/69159111-0e137e80-0aad-11ea-8b49-c19f963174f4.png)
![Screenshot from 2019-11-19 09-14-14](https://user-images.githubusercontent.com/39393023/69159112-0e137e80-0aad-11ea-8a23-e5db3c883837.png)

2. If you navigate the robot using 2D navigation marker in Rviz, you can see the robot avoiding a newly added obstacle:
![Screenshot from 2019-11-16 15-59-48](https://user-images.githubusercontent.com/39393023/69159343-63e82680-0aad-11ea-9315-cf4736c4cac7.png)
3. To see the robot simulation in the customized house environment with the new obstacles, run:
```
$roslaunch hw4 task4.launch add_obstacle:=true
```

## Question 5 Autonomous Gmapping Navigation

In this question, autonomous Gmapping Navigation is achieved using a custom node. By design, the node is capable of mapping the whole house,  by generating a random unmapped location within the house and map on the way to that location. Even though boundary is set so all goal destinations will not be outside of the house, the robot may get stuck when its planned path exits the house using the "side door" on the right side of the house in the above illustration. 
Therefore, future work needs to be done to address this issue. 

This node uses move_base action to get to a goal, which means when the robot is stuck, the goal needs to be changed so the robot might be able to move. The node's "unsticking" feature, changes the goal of travel if it takes the robot more than 10s to finish the action. From experiments, this is not entirely guranteed. However, one can generate a decent map using this method.

To run this node, do
```
$ roslaunch hw4 auto_gmapping.launch 
```
Below are the screenshots of two runs:
![Screenshot from 2019-11-19 21-14-27](https://user-images.githubusercontent.com/39393023/69206211-ad6a5d00-0b11-11ea-832d-3949557b8fb5.png)
![Screenshot from 2019-11-19 21-21-42](https://user-images.githubusercontent.com/39393023/69206564-a001a280-0b12-11ea-817a-01ab38ddcd25.png)






