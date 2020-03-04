### Problem Statement
Canadarm2 is a remote-controlled mechanical	arm aboard the International Space Station (ISS). The	robotic	arm	is	used	to	deploy,	capture	and	repair satellites, position astronauts,ma intain	equipment, and	move cargo. We need to develop Motion	Planning software for	a	simplified version of the	robotic	arm.  

### Mechanics of Canadarm
<a href="https://imgbb.com/"><img src="https://i.ibb.co/x7rBYnH/Screen-Shot-2020-03-04-at-3-06-15-pm.png" alt="Screen-Shot-2020-03-04-at-3-06-15-pm" width= 300px height = 200px  border="0"  title = "Fig 1"><a href="https://imgbb.com/"><img src="https://i.ibb.co/fNN6nCx/Screen-Shot-2020-03-04-at-3-06-18-pm.png" alt="Screen-Shot-2020-03-04-at-3-06-18-pm" width= 300px height = 200px border="0"></a><a href="https://ibb.co/h7406tp"><img src="https://i.ibb.co/f1yJgPm/Screen-Shot-2020-03-04-at-3-06-23-pm.png" alt="Screen-Shot-2020-03-04-at-3-06-23-pm" width= 200px height = 200px border="0"></a>

<br> 
The simplified Canadarm operates in a 2D workspace. In particular, the 2D workspace is a plane, represented as [0, 1] x [0, 1]  and is populated by rectangular obstacles. In addition, there are grapple point(s) which the end effectors of the robotic arm can attach to. One of the end effectors must be grappled to a grapple point at any time. The exact dimensions and positions of each obstacle and the number and position of the grapple points in the environment are known prior to execution. Fig.1 illustrates this environment. The robotic arm is composed of x links and x joints, where x is a non-negative integer, with two end effectors EE1 and EE2, which can attach onto grapple points. An example robotic arm with 3 links is shown in Fig.2 . Each link of the robot is a line segment attached to a joint. The link connected to the grappled end effector acts as a joint. Each joint is a rotational joint which means it can only rotate. A local coordinate system is attached to each joint. The co-ordinate system of the joint at the location of the grappled end effector, coincides with the coordinate system of the end effector. For the remaining joints, the x-axis is the line that coincides with the previous link. We define the angle of segment-i as the angle between link-i and the X axis of the coordinate system attached to joint-i. The joints are numbered relative to the grappled end-effector. Fig. 3 illustrates the rotational joints. In some tasks, the links are telescopic and can change length.This allows the robotic arm to more easily reach the grapple points. 

### Solver.py

Solver.py takes an test configuration as an input and returns a file with a path that the Canadarm can take to reach the destination without colliding with any obstructions. For detailed explanation you can read the Solution.docx

To get the Output path file, we need to run the command below in the terminal:  

      python solver.py testcases/4g3_m1.txt output.txt (Input file and Output file name needs to be given)

To run Visulization, we need to run the command below in the terminal: 

      python visualiser.py testcases/4g3_m1.txt output.txt (Input file and Output file name needs to be given)


The following files were pre-defined and imported from [Link](https://gitlab.com/3702-2019/assignment-2-support-code/)

**support/problem_spec.py**

This file contains the `ProblemSpec` class. This class serves the same purpose as the `SokobanMap` class from the first assignment - it can be used to represent the environment your agent operates in. The constructor for this class takes the filename of the input file as an argument, and handles parsing of the input file. The properties of the environment are stored as class variables within the `ProblemSpec` instance which can be accessed by your code.

Refer to the documentation at the start of this file for more details.

**support/robot_config.py**

This file contains the `RobotConfig` class. This class represents a particular configuration (state) the robot can be in. There are 2 helper methods which can be used to generate sample configurations using the reference frame of either EE1 or EE2. Additionally, this file provides a method for writing a solution (i.e. a list of `RobotConfig` objects forming a path) to an output file.

Refer to the documentation at the start of this file for more details.

**support/obstacle.py**

This file contains the `Obstacle` class, a simple class representing a rectangular obstacle.

Refer to the documentation at the start of this file for more details.

**support/angle.py**

This file contains the `Angle` class, representing an angle. This class behaves like a normal floating point number, supporting addition, subtraction, multiplication by scalar, division by scalar, negation, equality and comparison. Constructor accepts degrees or radians, and value can be accessed as degrees or radians. Automatically keeps value in the range of -pi and pi.

We suggest using this class when working with angles to avoid having to manually keep the angle within the correct range.

This class also contains static methods for performing trigonometric operations on `Angle` objects.

Refer to the documentation at the start of this file for more details.

**tester.py**

This file is a script which takes an input file and a solution file as input, and determines whether the solution file is valid. This script will be used to verify your solutions during your demonstration. Your should ensure that your solution files match the standard required by the `tester.py` script. For valid solution files, the script will output "Testcase solved successfully!".

Refer to the documentation at the start of this file for more details.

**visualiser.py**

This file is a GUI program which takes an input file and (optionally) a solution file as input, and provides a graphical representation of the input file, and animates the path provided in the solution file (if a solution file was given).

Refer to the documentation at the start of this file for more details.

**testcases**

Example input files for you to test your solver on. You should make sure that your solver is able to produce valid solution files (verified by `tester.py`) for these inputs.



Source Credits: UQ COMP 7702 coursework


