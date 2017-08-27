#include "poseestimation.h"
#include <stdexcept>
using namespace clubster;

void Alignment3DProblem::setupProblem()
{
    // check if source and target features are of the same length
    if (m_sourceFeatures.size()!=m_targetFeatures.size())
        throw std::runtime_error("Source and target features must be of the same size.");

    const auto num_feats = m_sourceFeatures.size();
    for (size_t i=0; i<num_feats; i++)
    {
        ceres::CostFunction * cf = new ceres::AutoDiffCostFunction<Alignment3DCost, ceres::DYNAMIC,
                4, 3, 3>(new Alignment3DCost(m_sourceFeatures[i], m_targetFeatures[i]),
                         3);
        ceres::LossFunction * lf = new ceres::HuberLoss(1);
        if (cf != NULL)
        {
            this->AddResidualBlock(cf,
                                   lf,
                                   getRotation().coeffs().data(),
                                   getTranslation().data(),
                                   getScale().data());
        }
        else
        {
            delete lf;
        }
    }

    // set rotation parameterization for the quaternion
    this->SetParameterization(getRotation().coeffs().data(),
                              new ceres::EigenQuaternionParameterization());
}

void Alignment3DProblem::solveProblem(const bool debug_mode)
{
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = debug_mode;
    ceres::Solver::Summary summary;
    Solve(options, this, &summary);

    if (debug_mode)
    {
        std::cout << summary.BriefReport() << std::endl;
    }
}

