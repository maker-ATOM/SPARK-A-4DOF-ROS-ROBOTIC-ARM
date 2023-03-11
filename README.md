# SPARK-A-4DOF-ROS-ROBOTIC-ARM

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#About">About</a></li>
    <li><a href="#Coordinate-system-and-References-for-FK-and-IK">Coordinate system and References for FK and IK</a></li>
    <li><a href="#Using-this-Project">Using this Project</a></li>
    <li><a href="#Description-of-src-files">Description of src files</a></li>
    <li><a href="#Project-Status">Project Status</a></li>
    <li><a href="#Project-Upgradation">Project Upgradation</a></li>
  </ol>
</details>

## About
The objective of this project is to provide a software platform for implementation of mathematical concepts like Forward Kinematics(FK), Inverse Kinematics(IK), Trajectory planning using a Robotic Arm in ROS.

## Coordinate system and References for FK and IK

Before controlling the robot it is advisable to understand the coordinate system and references of the robot.

**_NOTE:_** The coordinate system of robot is completely different from coordinate system of the Gazebo environment. Do not mix them.
 
### Axis and Grid

- The X-axis is defined along the horizontal direction in which the positive end is oriented towards left when viewed from a side view perspective.
- The Y-axis is defined along the vertical direction in which the positive end is oriented upwards when viewed from a side view perspective.
- Z-axis is defined by the right-hand-rule.
<p>

- Each square on the checkered grid has a length of 0.25 units.

![Axis and Grid](/images/axis_grid.png)

### Joints and Links
- Cylindrical part is the  <mark style="background-color: #FFA500"> base_link</mark>  which is fixed to the ground. This is not considered as a part of the robot.

- The initial link, denoted as <mark style="background-color: #FFFF00">Link A</mark>, is interconnected to the base_link through a revolute joint rotating around the y-axis, which is identified as Rev 1.
- The second link, denoted as <mark style="background-color: #00FF00">Link B</mark>, is connected to Link A through a revolute joint rotating around the z-axis, which is identified as Rev 2.
- The third link, denoted as <mark style="background-color: #30D5C8">Link C</mark>, is connected to Link B through a revolute joint rotating around the z-axis, which is identified as Rev 3.
- The fourth link, <mark style="background-color: #0000FF">Link C</mark>, designated as End-effector is connected to Link C through a revolute joint rotating around the z-axis, which is identified as Rev 4.

![Joints and Links](/images/joints_links.png)


### Parameters

- The x and y values utilized in (IK) are measured from the origin of the Rev1 joint to the end-effector.
- Phi (ϕ) denotes the angle measured of the end-effector in relation to the positive y-axis.
- The angle q0 is measured by Link B with respect to the positive y-axis.
- The angle q1 is measured by Link C with respect to the axis of Link B.
- The angle q2 is measured by Link D, which represents the end-effector, with respect to the axis of Link C.
- The angle q` represents the measurement of Link A's rotation around the y-axis (not depicted in the picture).

![Parameters](/images/parameters.png)

## Using this Project
- Move into your workspace/src folder
```
cd ~/catkin_ws/src/
```

- Clone the repository in your workspace
```
git clone https://github.com/maker-ATOM/SPARK-A-4DOF-ROS-ROBOTIC-ARM
```

- Perform make and build through catkin
```
cd ~/catkin_ws
catkin_make
```

Launch display.launch file to visualize joints anf transforms
```
roslaunch SPARK-A-4DOF-ROS-ROBOTIC-ARM display.launch 
```
Move the sliders to observe the behavior of joint angles.

![Rivz](/images/Rviz.gif
)
Kill the process by pressing `ctrl+c`


Launch gazebo.launch to simulate the Robotic Arm
```
roslaunch SPARK-A-4DOF-ROS-ROBOTIC-ARM gazebo.launch 
```

![Gazebo startup](/images/gazebo_startup.png)


The camera of gazebo is configured to display orthographic projection of side view. You can change by clicking middle mouse button and dragging.

![Gazebo Orientation](/images/gazebo_oreintation.png)


Open a new tab in terminal (`ctrl+shift+t`) and run square.py
```
rosrun SPARK-A-4DOF-ROS-ROBOTIC-ARM square.py
```
This program will command the end effector of Robotic Arm to traverse over a square.

![Draws a square](/images/square.gif)


## Description of src files
- FK
This program commands the arm to move at the position by defining joint angles mentioned in the argument list.
Arguments passed are [q`, q0, q1, q2] angles in radians.

```
rosrun SPARK-A-4DOF-ROS-ROBOTIC-ARM FK.py q` q0 q1 q2
```
Keeping q` = 0 restrains the arm movement in xy plane.

![FK](/images/FK.gif)


- IK_DOF2
This program does not include the end-effector as a component of the robotic arm. Consequently, only the end-point of Link C is commanded to move to the location specified in the argument.
The exclusion of Link D as a component of the robotic arm reduces the degrees of freedom to 2. In the event that the Rev 1 joint is not rotated, only two arguments, namely x and y, are passed.
```
rosrun SPARK-A-4DOF-ROS-ROBOTIC-ARM IK_DOF2.py x y
```
![IK_DOF2](/images/IK_DOF2.gif)

- IK_DOF3
This program accepts three parameters, namely [x, y, ϕ]. The values of x and y define the position to which the end-effector must move, while ϕ indicates the configuration that the robotic arm should attain upon reaching these points.
```
rosrun SPARK-A-4DOF-ROS-ROBOTIC-ARM IK_DOF3.py x y ϕ
```
![IK_DOF3](/images/IK_DOF3.gif)

- Square
This program instructs the end effector of the robotic arm to traverse over a square. A list containing the vertices of the square is stored, and based on the location of the vertices, it can be concluded that the sides are parallel to the axes. Thus, when following the path, only a single parameter (either x or y) will change. The corresponding variable is incremented, and the arm is commanded to reach that point.

## Project Status
- Currently all the src files work in 2D plane not changing the q` angle.
- Explicit IK and FK functions available that can be ported into any programs.

## Project Upgradation

- Add a python/ ROS GUI slider to update individual IK_DOF3 parameters [x,y,phi], providing a visual feedback for the movement in Gazebo.
- To develop an IK solution for 4 DOF giving arm the flexibility to reach any point in 3D space.
- To create an algorithm that can command the robotic arm to move in a straight path, irrespective of the location of the points.
- To generate an alternative quadratic path if a point on the linear path cannot be reached.
- Replace the 3D model of the arm with a 3D printable robot to to integrate ROS platform with a physical robot.