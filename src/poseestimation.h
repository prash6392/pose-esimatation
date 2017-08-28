#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "features3d.h"
#include "rigidtransformation.h"
#include "glog/logging.h"

namespace clubster {

    class Alignment3DCost
    {
    public:
        Alignment3DCost(const Point3D & sp, const Point3D & tp) :
            m_source_pt(sp),
            m_target_pt(tp)
        {}

        template <typename T>
        bool operator()(const T * const rot, const T*const trans, const T*const scale, T*residuals)
        const
        {
            Eigen::Map<const Eigen::Quaternion<T>> rq(rot);
            // set source point
            auto sp = m_source_pt.getPointAsVec();
            // set target point
            auto tp = m_target_pt.getPointAsVec();

            // rotate and translate point
            auto temp = rq*sp.cast<T>();
            temp[0] += trans[0];
            temp[1] += trans[1];
            temp[2] += trans[2];
            // scale point accordingly
            temp[0] *= scale[0];
            temp[1] *= scale[1];
            temp[2] *= scale[2];

            residuals[0] = tp.cast<T>()[0]-temp[0];
            residuals[1] = tp.cast<T>()[1]-temp[1];
            residuals[2] = tp.cast<T>()[2]-temp[2];

            return true;
        }

    private:
        const Point3D & m_source_pt;
        const Point3D & m_target_pt;
    };

    class Alignment3DProblem : public ceres::Problem
    {
    public:
        Alignment3DProblem(const Features3D & sf, const Features3D & tf) :
            m_sourceFeatures(sf),
            m_targetFeatures(tf),
            m_rotquat(Eigen::Quaterniond::Identity()),
            m_trans(0, 0, 0),
            m_scale(1, 1, 1)
        {}
        void setupProblem();
        void solveProblem(const bool debug_mode=false);
        Eigen::Quaterniond & getRotation() {return m_rotquat;}
        const Eigen::Quaterniond & getRotation() const {return m_rotquat;}
        Eigen::Vector3d & getTranslation() {return m_trans;}
        const Eigen::Vector3d & getTranslation()const {return m_trans;}
        Eigen::Vector3d & getScale() {return m_scale;}
        const Eigen::Vector3d & getScale() const {return m_scale;}
    private:
        const Features3D & m_sourceFeatures;
        const Features3D & m_targetFeatures;
        Eigen::Quaterniond m_rotquat;
        Eigen::Vector3d m_trans;
        Eigen::Vector3d m_scale;
    };
}


#endif // POSEESTIMATION_H
