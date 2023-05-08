Download Link: https://assignmentchef.com/product/solved-robotics-homework1-forward-kinematicsfk
<br>
The goal of homework1 is to practice and implement what you have learned in the recent courses. In the following exercises, you will implement your own <strong>forward kinematics(FK)</strong> algorithm step by step and test the algorithm in simulation using a <strong>Franka Panda 7DoF</strong> arm.

<h1>Exercise 0</h1>

Let’s take a look at what we have now. You will get:

<ol>

 <li>py : this file is a template <strong>where you need to write your own code</strong>.</li>

 <li>py : build a simulation environment. <strong>This file does not need to be modified</strong>, but you can call some functions provided for you.</li>

 <li>py : this file provides some demonstrations to show whether your algorithms work well in the simulation by calling functions in robot_sim.py and core.py .</li>

 <li>Folder /robot_model : contains the URDF files and its meshes. <strong>Do not change anything in this folder.</strong></li>

</ol>

Then you need to install the dependencies. Python version 3.7 or 3.8 is recommended. It is recommended to use <em>conda</em> and virtual environment to configure the programming environment.

The third-party libraries you need are: <em>pybullet</em> and <em>numpy</em>. For <em>numpy</em> version, I am using version

1.20.1. New versions and some slightly older versions are also available.

For pybullet, just run:

<h1>Exercise 1: Rigid-Body Motions 【</h1>

You need to implement some basics functions of rigid-body motions.

<strong>NOTE1</strong>: write your code in the template file core.py . <strong>DO NOT MODIFY THE FUNCTION NAME!!!</strong> Otherwise the autotest program will not work and you will lose your score &#x1f642;

<strong>NOTE2</strong>: <em>The input and output of the function are numpy arrays!</em>

Here is what you need to finish:

<ol>

 <li>invR=rot_inv(R) : Inverts a rotation matrix, use the property of rotation matrix.</li>

</ol>

input: R: Rotation matrix R return: The inverse of R

<ol start="2">

 <li>so3mat=vec_to_so3(omg) : Converts a 3-vector to an so(3) representation.</li>

</ol>

input: omg: A 3-vector return: The corresponding 3 × 3 skew-symmetric matrix in so(3).

<ol start="3">

 <li>omg=so3_to_vec(so3mat) : Converts an so(3) representation to a 3-vector.</li>

</ol>

input: so3mat: A 3 × 3 skew-symmetric matrix (an element of so(3)).

return: The corresponding 3-vector.

<ol start="4">

 <li>[omghat,theta]=axis_ang3(expc3) : Converts a 3-vector of exponential coordinates for rotation into axis-angle form input: expc3: A 3-vector of exponential coordinates for rotation</li>

</ol>

return1: omghat: The corresponding unit rotation axis

return2: theta: The corresponding rotation angle

<ol start="5">

 <li>R=matrix_exp3(so3mat) : Computes the matrix exponential of a matrix in so(3) input: so3mat: An so(3) representation of exponential coordinates for rotation, . return: The that is achieved by rotating about  by  from an initial orientation .</li>

 <li>R=matrix_log3(R) : Computes the matrix logarithm of a rotation matrix input: R: Rotation matrix.</li>

</ol>

return: so3mat: The corresponding so(3) representation of exponential coordinates.

<ol start="7">

 <li>T=rp_to_trans(R,p) : Converts a rotation matrix and a position vector into homogeneous transformation matrix. input1: R: Rotation matrix.</li>

</ol>

input2: p: A 3-vector

return: T: The corresponding homogeneous transformation matrix .

<ol start="8">

 <li>[R,p]=TransToRp(T) : Converts a homogeneous transformation matrix into a rotation matrix and position vector.</li>

</ol>

input: T: Transformation matrix.

return1: R: The corresponding rotation matrix.

return2: p: The corresponding position.

<ol start="9">

 <li>invT = trans_inv(T) : Inverts a homogeneous transformation matrix.</li>

</ol>

input: T: Transformation matrix.

return: invT: Inverse of T.

<ol start="10">

 <li>se3mat=vec_to_se3(V) : Converts a spatial velocity vector into a 4×4 matrix in se3.</li>

</ol>

input: V: A 6-vector (e.g. representing a twist).

return: se3mat: The corresponding 4 × 4 se(3) matrix.

<ol start="11">

 <li>V = se3_to_vec(se3mat) : Converts an se3 matrix into a spatial velocity vector.</li>

</ol>

input: se3mat: A 4 × 4 se(3) matrix. return: V: The corresponding 6-vector.

<ol start="12">

 <li>AdT = adjoint(T) : Computes the adjoint representation of a homogeneous transformation matrix.</li>

</ol>

input: T: Transformation matrix.

return: AdT: The corresponding 6 × 6 adjoint representation .

<ol start="13">

 <li>S=screw_to_axis(q, s, h): Takes a parametric description of a screw axis and converts it to a normalized screw axis.</li>

</ol>

input1: q: A point              lying on the screw axis. input2: An unit vector              in the direction of the screw axis.

input3: The pitch               (linear velocity divided by angular velocity) of the screw axis. return: S: The corresponding normalized screw axis .

<ol start="14">

 <li>[S,theta] = axis_ang6(expc6): Converts a 6-vector of exponential coordinates into screw axis-angle form.</li>

</ol>

input: expc6: A 6-vector of exponential coordinates for rigid-body motion, return1: S: The corresponding normalized screw axis S. return2: theta: The distance traveled along/about S.

<ol start="15">

 <li>T=matrix_exp6(se3mat) : Computes the matrix exponential of an se3 representation of exponential coordinates.</li>

</ol>

input: se3mat: An se(3) representation of exponential coordinates for rigid-body motion return: T: The    that is achieved by traveling along/about the screw axis  a distance  from an initial configuration .

<ol start="16">

 <li>se3mat=matrix_log6(T) : Computes the matrix logarithm of a homogeneous transformation matrix.</li>

</ol>

input: T: Transformation matrix. return: se3mat: The corresponding se(3) representation of exponential coordinates.

That’s the end of exercise1. You may find this list is a bit long, but these implementations are quite simple. So, don’t worry &#x1f642;

<h1>Exercise 2: Forward Kinematics</h1>

Now is time to use the code in exercise 1 to perform your own <strong>forward kinematics(FK)</strong> algorithm. Here are the functions you need to implement in the template:

<ol>

 <li>T=FK_in_body(M, Blist, thetalist): Computes forward kinematics in the body frame for an open chain robot.</li>

</ol>

input1: M: The home configuration of the end-effector.

input2: Blist: The joint screw axes in the end-effector frame when the manipulator is at the home position. input3: thetalist: A list of joint coordinate values.

return: The         representing the end-effector frame when the joints are at the specified coordinates.

<ol start="2">

 <li>T=FK_in_space(M, Slist, thetalist) : Computes forward kinematics in the space frame for an open chain robot.</li>

</ol>

input1:M: The home configuration of the end-effector.

input2: Slist: The joint screw axes in the space frame when the manipulator is at the home position. input3: thetalist: A list of joint coordinate values.

return: The  representing the end-effector frame when the joints are at the specified coordinates.

<h1>Exercise 3: Play with the demo</h1>

Congratulations! You have completed your coding part. It‘s time to play with a robot arm! (in simulation)

Run the code in demo.py , especially using function: FK_test(robot, physicsClientId)

In this demo, you will send commands to set the position of each joint and see whether the real configuration caused by commands is the same as the result of your analytical calculation using your own FK algorithm.

After running the demo, you will see a robot(Franka Panda Arm) like this.

You can press the CTRL and left mouse button and drag the mouse to change the view angle. You can also zoom in or out by sliding the mouse wheel.

If your FK algorithm is correct, you will find something like this:

This is the end of homework1~