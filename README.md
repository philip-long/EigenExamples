# Eigen

"Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms."

The motivation of this page is to show some Eigen example calls.

## Eigen::Vector3d()

```cpp
Eigen::Vector3d pos;
pos << 0.0,-2.0,1.0;
Eigen::Vector3d pos2 = Eigen::Vector3d(1,-3,0.5);
pos = pos+pos2;

Eigen::Vector3d pos3 = 	Eigen::Vector3d::Zero();

pos.normalize();
pos.inverse();

//Dot and Cross Product
Eigen::Vector3d v(1, 2, 3);
Eigen::Vector3d w(0, 1, 2);

double vDotw = v.dot(w); // dot product of two vectors
Eigen::Vector3d vCrossw = v.cross(w); // cross product of two vectors

```

## Eigen::Quaterniond()

```cpp
Eigen::Quaterniond rot;

rot.setFromTwoVectors(Eigen::Vector3d(0,1,0), pos);

Eigen::Matrix<double,3,3> rotationMatrix;

rotationMatrix = rot.toRotationMatrix();

Eigen::Quaterniond q(2, 0, 1, -3);
std::cout << "This quaternion consists of a scalar " << q.w()
<< " and a vector " << std::endl << q.vec() << std::endl;

q.normalize();

std::cout << "To represent rotation, we need to normalize it such
that its length is " << q.norm() << std::endl;

Eigen::Vector3d vec(1, 2, -1);
Eigen::Quaterniond p;
p.w() = 0;
p.vec() = vec;
Eigen::Quaterniond rotatedP = q * p * q.inverse();
Eigen::Vector3d rotatedV = rotatedP.vec();
std::cout << "We can now use it to rotate a vector " << std::endl
<< vec << " to " << std::endl << rotatedV << std::endl;

// convert a quaternion to a 3x3 rotation matrix:
Eigen::Matrix3d R = q.toRotationMatrix();

std::cout << "Compare with the result using an rotation matrix "
<< std::endl << R * vec << std::endl;

Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
Eigen::Quaterniond b = Eigen::Quaterniond::Identity();
Eigen::Quaterniond c;
// Adding two quaternion as two 4x1 vectors is not supported by the Eigen API.
//That is, c = a + b is not allowed. The solution is to add each element:

c.w() = a.w() + b.w();
c.x() = a.x() + b.x();
c.y() = a.y() + b.y();
c.z() = a.z() + b.z();

```

## Eigen Euler angle and fixed axis rotation (roll, pitch, yaw)
```cpp
// we use fixed axis rotation(rpy) to get rotation matrix

//  roll, pitch, yaw = XYZ fixed axis rotation = Eurler angle ZYX
Eigen::Quaterniond q = AngleAxisd(0.1*M_PI, Vector3d::UnitX())
                      * AngleAxisd(0.2*M_PI, Vector3d::UnitY())
                      * AngleAxisd(0.3*M_PI, Vector3d::UnitZ());

Eigen::Matrix3d R = q.toRotationMatrix();


// roll pitch yaw to rotation matrix directly
Matrix3d R = AngleAxisd(PI, Vector3d::UnitX())
        * AngleAxisd(0, Vector3d::UnitY())
        * AngleAxisd(0, Vector3d::UnitZ());
// return euler angles as sequence XYZ Euler angle from rotatiton matrix
Vector3d euler_angles = R.eulerAngles(0,1,2); // 0: x axis, 1: y axis, 2: z axis

// recompute the rotation matrix from eurler angles
Matrix3d n;
n = AngleAxisd(euler_angles[0], Vector3d::UnitX())
   *AngleAxisd(euler_angles[1], Vector3d::UnitY())
   *AngleAxisd(euler_angles[2], Vector3d::UnitZ());


// return ZXZ euler angles from rotation matrix
Vector3d ea = R.eulerAngles(2, 0, 2); // 2,0,2 = ZXZ
```

### References
- [Creating a rotation matrix with pitch, yaw, roll using Eigen](https://stackoverflow.com/a/26297599)
- [Roll pitch and yaw from Rotation matrix with Eigen Library](https://stackoverflow.com/a/27880029)
- [Eigen Geometry module](https://eigen.tuxfamily.org/dox/group__Geometry__Module.html)



## Eigen::Affine3d()  
Affine3d is a Pose type message( contains a Vector 3d and Quaterniond/RotationMatrix). It is great for computing several subsequent transformations.

```cpp
Eigen::Affine3d aff = Eigen::Affine3d::Identity();

aff.translation() = pos;
aff.translation() = Eigen::Vector3d(0.7,0.4,1.1);
aff.linear() = rot.toRotationMatrix();

aff.translate(Eigen::Vector3d(0,0.0,0.03));
aff.pretranslate(pos);

aff.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)));
aff.prerotate(rot.toRotationMatrix());

Eigen::MatrixXd normalMatrix = aff.linear().inverse().transpose();

```

To get the translation and rotation portions:

```cpp
pos = aff.translation();
rot = aff.linear();
aff.linear().col(0);
```

## Eigen::MatrixXd()

```
//Transpose and inverse:
Eigen::MatrixXd A(3, 2);
A << 1, 2,
     2, 3,
     3, 4;

Eigen::MatrixXd B = A.transpose();// the transpose of A is a 2x3 matrix
// computer the inverse of BA, which is a 2x2 matrix:
Eigen::MatrixXd C = (B * A).inverse();
C.determinant(); //compute determinant
Eigen::Matrix3d D = Eigen::Matrix3d::Identity();

Eigen::Matrix3d m = Eigen::Matrix3d::Random();
m = (m + Eigen::Matrix3d::Constant(1.2)) * 50;
Eigen::Vector3d v2(1,2,3);

cout << "m =" << endl << m << endl;
cout << "m * v2 =" << endl << m * v2 << endl;

//Accessing matrices:
Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(7, 9);
std::cout << "The fourth row and 7th column element is " << A2(3, 6) << std::endl;

Eigen::MatrixXd B2 = A2.block(1, 2, 3, 3);
std::cout << "Take sub-matrix whose upper left corner is A(1, 2)"
<< std::endl << B2 << std::endl;

Eigen::VectorXd a2 = A2.col(1); // take the second column of A
Eigen::VectorXd b2 = B2.row(0); // take the first row of B2

Eigen::VectorXd c2 = a2.head(3);// take the first three elements of a2
Eigen::VectorXd d2 = b2.tail(2);// take the last two elements of b2

```
## Eigen::Transform()
Class for rigid body transformations

```cpp
    Eigen::Vector3d t1(1, 2, -1);
    Eigen::Vector3d t5;
    Eigen::Vector3d t4(10,0.52,0.5);
    Eigen::Quaterniond q1(2, 0, 1, -3); // Quaterniond(w,x,y,z)
    Eigen::Quaterniond q4(2, 5,0.5, -5);
    Eigen::Transform<double,3,Eigen::Affine>  T1,T2,T3,T4,T5;
    T1.translation()=t1;
    T4.translation()=t4;
    q1.normalize();
    q4.normalize();
    T1.linear()=q1.toRotationMatrix(); // define rotation matrix
    T4.linear()=q4.toRotationMatrix(); // define rotation matrix

    std::cout<<"T1 =    "<<T1.linear().row(0)<<" "<<T1.translation().row(0)<<std::endl;
    std::cout<<"        "<<T1.linear().row(1)<<" "<<T1.translation().row(1)<<std::endl;
    std::cout<<"        "<<T1.linear().row(2)<<" "<<T1.translation().row(2)<<std::endl;
    std::cout<<"        "<<0.0 <<" "<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;

    std::cout<<"T4 =    "<<T4.linear().row(0)<<" "<<T4.translation().row(0)<<std::endl;
    std::cout<<"        "<<T4.linear().row(1)<<" "<<T4.translation().row(1)<<std::endl;
    std::cout<<"        "<<T4.linear().row(2)<<" "<<T4.translation().row(2)<<std::endl;
    std::cout<<"        "<<0.0 <<" "<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;

    // Obtain the inverse of matrix T1 i.e.
    // if T1 = [R1    P1]
    //         [0 0 0  1]

    // then T2 = [transpose(R1) -transpose(R1)*P1]
    //           [0 0 0              1           ]

    T2=T1.inverse(Eigen::TransformTraits::Affine) ;


    std::cout<<"T2 =    "<<T2.linear().row(0)<<" "<<T2.translation().row(0)<<std::endl;
    std::cout<<"        "<<T2.linear().row(1)<<" "<<T2.translation().row(1)<<std::endl;
    std::cout<<"        "<<T2.linear().row(2)<<" "<<T2.translation().row(2)<<std::endl;
    std::cout<<"        "<<0.0 <<" "<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;

    // Verify inverse is identity
    T3=T2*T1;


    std::cout<<"T3 =    "<<T3.linear().row(0)<<" "<<T3.translation().row(0)<<std::endl;
    std::cout<<"        "<<T3.linear().row(1)<<" "<<T3.translation().row(1)<<std::endl;
    std::cout<<"        "<<T3.linear().row(2)<<" "<<T3.translation().row(2)<<std::endl;
    std::cout<<"        "<<0.0 <<" "<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;

    // Spatial Transformations
    T5=T2*T4*T1;

    std::cout<<"T5 =    "<<T5.linear().row(0)<<" "<<T5.translation().row(0)<<std::endl;
    std::cout<<"        "<<T5.linear().row(1)<<" "<<T5.translation().row(1)<<std::endl;
    std::cout<<"        "<<T5.linear().row(2)<<" "<<T5.translation().row(2)<<std::endl;
    std::cout<<"        "<<0.0 <<" "<<0.0<<" "<<0.0<<" "<<1.0<<std::endl;

    // Spatial transformation of a translation vector
    t5=T5*t1;

    std::cout<<t5.col(0)<<std::endl;
    
```
# Eigen Beginner

Include some **header files**, **Basic matrix manipulation**, please see [Modules and Header files](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html).

```cpp
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

int main()
{
  // Matrix
  MatrixXd m = MatrixXd::Random(3,3);
  m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
  cout << "m =" << endl << m << endl;

  // vector
  VectorXd v(3);
  v << 1, 2, 3;
  cout << "m * v =" << endl << m * v << endl;
}
```

```cpp
MatrixXd m(2,2);
m(0,0) = 3;
m(1,0) = 2.5;
m(0,1) = -1;
m(1,1) = m(1,0) + m(0,1);

// print matrix
std::cout << m << std::endl;
```

```cpp
// matrix basic operator
m2 = m + m1
m2 = m*m1
```



## References

- [Eigen Webpage](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Eigen Space transformations](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html)
- [Eigen Getting started](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [Eigen Quick reference guide](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)
- [Eigen Advanced initialization](http://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html)
- [Dense matrix and array manipulation](http://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__chapter.html)
- [Eigen Library Tutorial](http://www.cc.gatech.edu/classes/AY2015/cs4496_spring/Eigen.html)


# Eigen transformations to KDL Frame and ROS Pose message


## References
 - [eigen_conversions](https://github.com/ros/geometry/tree/indigo-devel/eigen_conversions/src)
 - [kdl_conversions](https://github.com/ros/geometry/tree/indigo-devel/kdl_conversions)



# Space Transformations with TF transformations Python interface

## TF Homogeneous transformations matrix Python interface
```py
from tf.transformations import *

# all matrix output from tf.transformations is 4*4 Homogeneous Transformation Matrices

# define directly from numpy array
H = numpy.array([[1,0,0,0], [0,1,0,0], [0, 0, 1, 0], [0,0,0,1]])

# unit Homogeneous Transformation Matrix
I = identity_matrix()

# translation matrix (NOTE: actually is rotation matrix is unit 3*3 matrix, and position vector is (0,0,0,1))
Position = translation_matrix((0.1, 0.2, 0.3))

# general Homogeneous Transformation matrix with R and P
Rotation = I
M = concatenate_matrices(Position, Rotation)

# sequence transformation matrix muliplication
Result = concatenate_matrices(M, M1, M2, M3, M4, M5)
```

Quaternion to Rotation Homogeneous matrix

**NOTE:** quaternion only have four number (x,y,z,w) = xi + yj + zk + w
```py
# define quaternion directly from tuple, list, or numpy arrays.
q = [0,0,0,1]

# quaternion from angle-axis
alpha = 0.123
xaxis = (1, 0, 0)
qx = quaternion_about_axis(alpha, xaxis)


# quaternion multiply which align with rotation matrix multiply
q = quaternion_multiply(qx, qy)
```

More deatil, please see [tf/transformations in ROS/geometry github](https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py).


## Wrap Homogeneous transformations matrix to `PyKDL` Frame and ROS pose message
```py
# transform homogeneous_matrix (4*4 numpy arrary) to PyKDL Frame
kdl_frame = fromMatrix(homogeneous_matrix)

# PyKDL Frame to ROS Pose message
pose_msg = toMsg(kdl_frame)
```

More detail, please see  [tf_conversions/posemath](https://github.com/ros/geometry/blob/indigo-devel/tf_conversions/src/tf_conversions/posemath.py), [PyKDL - Frame transformations (Python)](http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29).


## KDL C++ interface
```cpp
KDL::Frame object_base_kdl_frame;
KDL::Frame base_kdl_frame;
KDL::Frame rel_kdl_frame;

rel_kdl_frame = KDL::Frame(KDL::Rotation::RPY(0, 0, 0),
                                KDL::Vector(0.060022, 0, 0));
object_base_kdl_frame = base_kdl_frame * rel_kdl_frame;
tf::poseKDLToMsg(object_base_kdl_frame, object_base_pose);
```

References：
- [KDL C++ API](http://docs.ros.org/indigo/api/orocos_kdl/html/geomprim.html)
- [KDL C++ Roation](http://docs.ros.org/indigo/api/orocos_kdl/html/classKDL_1_1Rotation.html)
- [KDL::Frame C++ Class Reference](http://docs.ros.org/indigo/api/orocos_kdl/html/classKDL_1_1Frame.html)



## Reference
- [tf/transformations in ROS/geometry github](https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py)
- [Frame transformations (Python)](http://wiki.ros.org/kdl/Tutorials/Frame%20transformations%20%28Python%29)
- [PyKDL API interface](http://docs.ros.org/diamondback/api/kdl/html/python/index.html)



# Transforms3d python package
[Transforms3d](http://matthew-brett.github.io/transforms3d/index.html) python package is a standalone and general space transform python package which can replace the ROS [tf](https://github.com/ros/geometry/tree/indigo-devel/tf) sub-module [transformations](https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py) about space transform. And it support `python3` and `python2`, and it only depend on `numpy`.

## Install transforms3d

```py
pip install transforms3d
```

## Homogeneous Affine Matrix

```py
import transforms3d as tf
import numpy as np

T = [20, 30, 40] # translations
R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]] # rotation matrix
Z = [2.0, 3.0, 4.0] # zooms

A = tf.affines.compose(T, R, Z)
```
output:

```py
array([[  0.,  -3.,   0.,  20.],
       [  2.,   0.,   0.,  30.],
       [  0.,   0.,   4.,  40.],
       [  0.,   0.,   0.,   1.]])
```

## quaternions

```py
# q = (w, x, y, z) = w+xi+yj+zk
q = [0, 1, 0, 0]

# quaternion to rotation matrix
M = tf.quaternions.quat2mat(q)
```


## Reference
- [Transforms3d](http://matthew-brett.github.io/transforms3d/index.html)
