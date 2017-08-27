#include "poseestimation.h"

using namespace clubster;

int main(int argc, char**argv)
{
    char * file_name=NULL;
    if (argc > 0)
    {
        file_name = argv[1];
    }
    if (file_name == NULL)
    {
        std::runtime_error("Filename cannot be empty.");
    }
    // read features from file
    Features3D f1(file_name);
    // std::cout << "original feature set:\n" << f1 << std::endl;
    // setup dummy rigid transform
    RigidTransformation rt;
    Eigen::Vector3d tvec;
    tvec.setOnes();
    rt.setTranslation(tvec);
    std::cout << "Rigid Transformation:\n" << rt << std::endl;

    Features3D f2(f1);
    // transform all points in this feature set
    f2.transformFeatureSet(rt);
    // std::cout << "transformed feature set: " << f2 << std::endl;

    // Solve this 3d Alignment problem
    Alignment3DProblem ap(f1, f2);
    ap.setupProblem();
    ap.solveProblem();
    std::cout << "Solution:\n" << "Rotation: " << ap.getRotation().toRotationMatrix() <<
                 "\nTranslation: " << ap.getTranslation() <<
                 "\nScale: " << ap.getScale() << std::endl;
}
