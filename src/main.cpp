#include "poseestimation.h"
#include <time.h>

using namespace clubster;

int main(int argc, char**argv)
{
    char * file_name=NULL;
    // Check arguments.
    if (argc != 2)
    {
        throw std::runtime_error("PoseEstimation needs exactly two arguments. "
                                 "PoseEstimation path_to_feature_file");
        return -1;
    }
    else
    {
        file_name = argv[1];
    }
    if (file_name == NULL)
    {
        throw std::runtime_error("Filename cannot be empty.");
        return -1;
    }
    // set seed for random number generator.
    std::srand((unsigned int) time(0));

    // read features from file
    Features3D f1(file_name);
    // Initialize a random Rigid transform
    RigidTransformation rigid_transform;
    // translation for the rigid transform
    Eigen::Vector3d tvec;
    // scale for the rigid transform
    Eigen::Vector3d scale;
    tvec.setRandom();
    scale.setRandom();
    rigid_transform.setTranslation(tvec);
    rigid_transform.setScale(scale);
    std::cout << "Rigid Transformation:\n" << rigid_transform << std::endl;

    // Create a new feature set
    Features3D f2(f1);
    // transform all points in this new feature set f2.
    f2.transformFeatureSet(rigid_transform);

    // Solve the 3d Alignment problem - Finds the unknown rotation, translation and scale.
    Alignment3DProblem ap(f1, f2);
    ap.setupProblem();
    // Solve problem with debug prints enabled.
    ap.solveProblem(true);
    // Print out the estimated solution.
    std::cout << "Solution:\n" << "Rotation:\n" << ap.getRotation().toRotationMatrix() <<
                 "\nTranslation:\n" << ap.getTranslation() <<
                 "\nScale:\n" << ap.getScale() << std::endl;
}
