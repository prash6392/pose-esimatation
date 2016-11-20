#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "calculate_reprojection_error.h"

#define NUM_LANDMARKS           68
#define NUM_CAMERA_PARAMS       7

using namespace std;
using namespace ceres;

template<typename T>
void FscanfOrDie(FILE* fptr, const char* format, T* value) {
  int num_scanned = fscanf(fptr, format, value);
  if (num_scanned != 1) {
    LOG(FATAL) << "Invalid UW data file.";
  }
}


bool read_2d_correspondences_from_file(char * file_name, double * landmark_x_locs, double * landmark_y_locs)
{
    FILE *fp = fopen(file_name, "r");

    for (int i=0; i< NUM_LANDMARKS; i++)
    {
        FscanfOrDie(fp, "%lf", &landmark_x_locs[i]);
        FscanfOrDie(fp, "%lf", &landmark_y_locs[i]);
    }/*end for*/
    fclose(fp);

    return true;
}/*end read_2d_correspondences_from_file*/

bool read_3d_correspondences_from_file(char * file_name, double * mc)
{
    FILE * fp = fopen(file_name, "r");
    for (int i=0; i<NUM_LANDMARKS; i++)
    {
        FscanfOrDie(fp, "%lf", &mc[3*i]);
        FscanfOrDie(fp, "%lf", &mc[3*i+1]);
        FscanfOrDie(fp, "%lf", &mc[3*i+2]);
    }
    fclose(fp);
    return true;
}/*end read_3d_correspondences_from_file*/

void init_camera(double * camera)
{
    /*rotation*/
    camera[0] = 0.0;
    camera[1] = 0.0;
    camera[2] = 0.0;
    /*translation*/
    camera[3] = 0.0;
    camera[4] = 0.0;
    camera[5] = 0.0;
    /*focal length*/
    camera[6] = 0.0;
}/*end init_camera*/


void build_problem(double * obs_x, double * obs_y, double * model_corres, double* camera, Problem * problem)
{
    for (int i =0; i<NUM_LANDMARKS; i++)
    {
        CostFunction * cf;
        cf = SnavelyReprojectionError::Create(obs_x[i], obs_y[i]);
        LossFunction * lf = new HuberLoss(1.0);
        double * point = model_corres + (3*i);
        problem->AddResidualBlock(cf, lf, camera, point);
    }/*end for every landmark*/
}/*end build_problem*/

void solve_problem(Problem * problem)
{
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    
    Solver::Summary summary;
    Solve(options, problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

int main(int argc, char **argv)
{
    char * correspondences_2d_file;
    char * correspondences_3d_file;
    /*location of observed facial landmarks*/
    double landmark_x_locs[NUM_LANDMARKS];
    double landmark_y_locs[NUM_LANDMARKS];
    /*3D correspondences*/
    double model_correspondences[NUM_LANDMARKS*3];
    /*camera*/
    double camera[NUM_CAMERA_PARAMS];
    /*ceres problem*/
    Problem problem;

    if (argc != 3)
    {
        printf("call as estimate-camera-params.cpp 2d 3d\n");
        return 0;
    }
    correspondences_2d_file = argv[1];
    correspondences_3d_file = argv[2];

    printf("Processing file: %s\n", correspondences_2d_file);
    /*Read 2d correspondences from file and store in a vector*/
    read_2d_correspondences_from_file(correspondences_2d_file, 
                                        landmark_x_locs,
                                        landmark_y_locs);

    printf("Processing file: %s\n", correspondences_3d_file);
    /*Read 3d correspondence from file and store in a vector*/
    read_3d_correspondences_from_file(correspondences_3d_file,
                                        model_correspondences);

    /*Initialize the camera matrix*/
    init_camera(camera);

    printf("Initializations complete. Building bundle adjustment problem.\n");
    /*build a ceres bundle adjustment problem*/
    build_problem(landmark_x_locs, 
                    landmark_y_locs,
                    model_correspondences,
                    camera,
                    &problem);
    /*Come on !*/
    solve_problem(&problem);

    printf("Done!!\n");
    return 0;
}/*end main*/


