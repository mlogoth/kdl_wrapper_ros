# KDL Wrapper ROS
A C++ wrapper for KDL library. Set and return variables in type of Eigen.

# Usage - Avaliable Functions

## Initialization

```c++
// Constructors
KDLWrapper(std::string robot_description, ros::NodeHandle nh);
KDLWrapper(KDL::Tree kdl_tree, ros::NodeHandle nh);
```
Initialize the class of the KDL Wrapper by defining either a robot description name that is loaded on parameter server or a kdl tree.


```c++
// costruct the chain and inialize the solvers
bool init(const std::string &chain_root, const std::string &chain_tip, bool gravity);
```
Then define the tip and root link of the the chain. The gravity is by default enabled. If not set the gravity false.


## Forward Kinematics

```c++
 /*
* Forward Kinematics :
* */
int ComputeFK(...)
```
The function that computes the forward kinematics gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement, **or**,
    - std::vector with joints' displacement 
*  Return Variables:
    - Eigen::VectorXd (size 6) -> position x,y,z and euler roll,pitch,yaw, **or**,
    - Eigen::Vector3d and Eigen::Matrix3d -> position x,y,z and rotation matrix, **or**,
    - Eigen::Vector3d and Eigen::Quaterniond -> position x,y,z and quaternion vector


```c++
 /*
* Forward Kinematics Velocity:
* */
int ComputeFKVel(...)
```
The function that computes the forward kinematics and velocity gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and** 
    - Eigen::VectorXd with joints velocity 
*  Return variables:
    - Eigen::VectorXd (size 6) -> end effector linear and angular velocity and Eigen::Vector3d with end effector position and Eigen::Matrix3d with end effector orientation

## Inverse Kinematics

```c++
 /*
* Inverse Kinematics :
* */
int ComputeIK(...)
```
The function that computes tip_link inverse kinmeatics gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and** 
    - Eigen::VectorXd with tip_link pose (size 6) with euler angles, **or**,
    - Eigen::Vector3d &position_in **and** const Eigen::Matrix3d &rotation_in with the position and rotation matrix of tip_link, **or**
    - Eigen::Vector3d &position_in **and** const Eigen::Quaterniond &rotation_in with
    the positiona and quaternion vector of tip_link
*  Return variables:
    - Eigen::VectorXd  with joint displacement.

```c++
 /*
* Inverse Kinematics Velocity:
* */
int ComputeIKVel(...)
```
The function that computes tip_link inverse velocity kinmeatics gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and** 
    - Eigen::VectorXd with tip_link linear and angular velocity
*  Return variables:
    - Eigen::VectorXd with joints' velocity.
  

## Jacobian Computation

```c++
 /*
* Compute Jacobian:
* */
int ComputeJac(...)
```
The function that computes tip_link inverse kinmeatics gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement
*  Return variables:
    - Eigen::MatrixXd  with jacobian matrix.

```c++
 /*
* Compute Time Derivative of Jacobian:
* */
int ComputeJacDot(...)
```
The function that computes tip_link inverse velocity kinmeatics gets arguements:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and** 
    - Eigen::VectorXd with joints' velocity
*  Return variables:
    - Eigen::MatrixXd  with time derivative of the jacobian matrix.
  

## Robot Dynamics

### Mass Inertia Matrix
```c++
 /*
* Compute Mass Inertia Matrix:
* */
int ComputeMassMatrix(...)
```
The function that computes the mass inertia matrix:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement
*  Return variables:
    - Eigen::MatrixXd  with mass-inertia matrix in *joint space*.

```c++
 /*
* Compute Cartesian Space Mass Inertia Matrix:
* */
int ComputeMassMatrixCartesian(...)
```
The function that computes the mass inertia matrix:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement
*  Return variables:
    - Eigen::MatrixXd with mass-inertia matrix in *cartesian space*.
  

### Coriolis Vector
```c++
 /*
* Compute Coriolis Vector:
* */
int ComputeCoriolisVector(...)
```
The function that computes a vector tha contains the coriolis terms:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and**
    - Eigen::VectorXd with joints' velocity
*  Return variables:
    - Eigen::VectorXd with coriolis vector in *joint space*.

```c++
 /*
* Compute Coriolis Vector in cartesian space:
* */
int ComputeCoriolisVectorCartesian(...)
```
The function that computes a vector tha contains the coriolis terms:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and**
    - Eigen::VectorXd with joints' velocity
*  Return variables:
    - Eigen::VectorXd with coriolis vector in *cartesian space*.
  

  
### Gravity Vector
```c++
 /*
* Compute Gravity Vector
* */
int ComputeGravityVector(...)
```
The function that computes a vector tha contains the coriolis terms:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and**
*  Return variables:
    - Eigen::VectorXd with gravity vector in *joint space*.

```c++
 /*
* Compute Cartesian Gravity Vector:
* */
int ComputeGravityVectorCartesian(...)
```
The function that computes a vector tha contains the coriolis terms:
*  Input Variables:
    - Eigen::VectorXd with joints' displacement **and**
*  Return variables:
    - Eigen::VectorXd with gravity vector in *cartesian space*.

### Inverse Dynamics
```c++
/*
* Compute Inverse Dynamics:
* @param Eigen::VectorXd of q, qdot, qddot,   
* @return Eigen::VectorXd with the torques   
* */
int ComputeInverseDynamics(...);
```

## Other Functions
```c++
/*
* Compute Pseudo Inverse:
* @param Eigen::MatrixXd of matrix M    
* @return Eigen::MatrixXd with the pinverse of M   
* */
int pinv(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv);
```

```c++
/*
* Compute Euler Transformation Matrix :
* @param Eigen::Vector3d of euler vector, std::string with euler reprsentation     
* @return Eigen::Matrix3d with the euler Tf   
* */
int eulerTf(const Eigen::Vector3d &eu, Eigen::Matrix3d &Tf, const std::string &representation = "ZYX");
```
