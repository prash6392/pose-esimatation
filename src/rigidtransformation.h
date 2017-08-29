#ifndef RIGIDTRANSFORMATION_H
#define RIGIDTRANSFORMATION_H

#include <Eigen/Dense>

namespace clubster {

    class RigidTransformation
    {
    public:
        RigidTransformation();
        RigidTransformation(const RigidTransformation & other):
            m_rotmat(other.m_rotmat),
            m_trans(other.m_trans)
        {}
        void setTranslation(const Eigen::Vector3d & translation) {m_trans = translation;}
        void setRotation(const Eigen::Matrix3d & rotation) {m_rotmat = rotation;}
        void setScale(const Eigen::Vector3d & scale) {m_scale = scale;}
        Eigen::Vector3d & getTranslation() {return m_trans;}
        const Eigen::Vector3d & getTranslation() const {return m_trans;}
        Eigen::Vector3d & getScale() {return m_scale;}
        const Eigen::Vector3d & getScale() const {return m_scale;}
        Eigen::Matrix3d & getRotation() {return m_rotmat;}
        const Eigen::Matrix3d & getRotation() const {return m_rotmat;}
        Eigen::Vector3d transformPoint(const Eigen::Vector3d & point) const;
        friend std::ostream & operator<<(std::ostream & os, const RigidTransformation & rt)
        {
            os << "Rotation:\n" << rt.m_rotmat << "\nTranslation:";
            os << "\n" << rt.m_trans << "\n";
            os << "Scale:\n" << rt.m_scale << std::endl;
            return os;
        }
    private:
        Eigen::Matrix3d m_rotmat;
        Eigen::Vector3d m_trans;
        Eigen::Vector3d m_scale;
    };
}


#endif // RIGIDTRANSFORMATION_H
