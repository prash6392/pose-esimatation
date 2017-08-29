#include "rigidtransformation.h"

using namespace clubster;

RigidTransformation::RigidTransformation()
{
    // set rotation matrix to identity
    m_rotmat.setZero();
    m_rotmat(0, 0) = 1;
    m_rotmat(1, 1) = 1;
    m_rotmat(2, 2) = 1;
    // set translation to zero
    m_trans.setZero();
}

Eigen::Vector3d RigidTransformation::transformPoint(const Eigen::Vector3d & point)
const
{
    // rotate point
    Eigen::Vector3d vec = m_rotmat*point;
    // translate point
    Eigen::Vector3d res = vec + m_trans;
    // scale point
    for (int i=0; i<3; i++)
        res(i) = res(i)*m_scale(i);
    return res;
}
