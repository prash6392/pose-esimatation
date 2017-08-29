# 3D Pose estimation

## Introduction
Given two sets of points in R3 (3D space), the goal of 3D pose estimation is to find a
transformation that maps one set of 3D points to the other.

## Problem description
In this problem, we are given a set of 3D points which have been transformed
with an unknown transformation. The transformation consists of the following parameters.

    - Rotation R
    - Translation T
    - Scale S

Our goal is to estimate the parameters of this unknown transformation given a set of
correspondences in 3D.

## Understanding the unknowns
### Rotation R
The rotation R is a 3x3 matrix with 9 unknowns. However not all 3x3 matrices represent valid
rotations. Rotation matrices are unitary matrices and span a subspace of the space of all 3x3
matrices. The restricts R to the space of all unitary 3x3 matrices.

Rotations can be characterized with the help of unit quaternions. A unit quaternion represents
a rotation around a specific axis. They are however different from the axis-angle representation
of rotations. Here, rotations are represented with unit quaternions.

### Translation T
The translation vector T is a 3D vector denoting one translation for the x, y and z axes
respectively.

### Scale S
The scale S, considered here is an anisotropic 3D vector. It represents one scaling parameter
for the x, y and z axes respectively.

## Optimization
The 3D alignment problem is formulated as a least squares problem where the squared euclidean
distance between the source and the target points is to be minimized. In order to be robust
to outliers, the huber loss function is used.

## References
[1] Ceres Solver (http://ceres-solver.org/) Sameer Agarwal and Keir Mierle and Others.




