#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <math.h>
#include "calculate_reprojection_error.h"

#define NUM_LANDMARKS           68
#define NUM_CAMERA_PARAMS       9

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
    camera[6] = -40.0;
    /*image plan translation*/
    camera[7] = 0.0;
    camera[8] = 0.0;
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
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.max_linear_solver_iterations = 500;
    Solver::Summary summary;
    Solve(options, problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

void get_rotation_matrix(double * camera_params, double * rotation_matrix)
{
    double rod_rot_vector[3] = {camera_params[0], camera_params[1], camera_params[2]};
/*
    static const double kOne = 1.0;
    double theta = DotProduct(rod_rot_vector, rod_rot_vector);
    printf("Dot product result: %f\n", theta);

    double theta2 = std::sqrt(theta);
    printf("Sqrt of theta: %f\n", theta2);

    const double wx = rod_rot_vector[0] / theta;
    const double wy = rod_rot_vector[1] / theta;
    const double wz = rod_rot_vector[2] / theta;

    const double costheta = std::cos(theta);
    const double sintheta = std::sin(theta);

    double *R = rotation_matrix;

    R[0] =     costheta   + wx*wx*(kOne -    costheta);
    R[1] =  wz*sintheta   + wx*wy*(kOne -    costheta);
    R[2] = -wy*sintheta   + wx*wz*(kOne -    costheta);
    R[3] =  wx*wy*(kOne - costheta)     - wz*sintheta;
    R[4] =     costheta   + wy*wy*(kOne -    costheta);
    R[5] =  wx*sintheta   + wy*wz*(kOne -    costheta);
    R[6] =  wy*sintheta   + wx*wz*(kOne -    costheta);
    R[7] = -wx*sintheta   + wy*wz*(kOne -    costheta);
    R[8] = costheta + wz*wz*(kOne - costheta);
*/
    AngleAxisToRotationMatrix(rod_rot_vector, rotation_matrix);

}

void write_estimated_parameters_to_file(char * file_name, double * camera_params, double * rot_mat)
{
    FILE *fp = fopen(file_name, "w");

    printf("Writing results to file: %s\n", file_name);
    if ((fp == NULL))
    {
        printf("Unable to open file %s\n!", file_name);
    }
    int bytes_written = fprintf(fp, "%.06f\n", rot_mat[0]);
    /*Write elements of the rotation matrix*/
    for (int i=1;i<9;i++)
    {
        bytes_written += fprintf(fp, "%.06f\n", rot_mat[i]);
    }
    /*Write the translation parameters and the focal length*/
    for (int i=3; i<9; i++)
    {
        bytes_written += fprintf(fp, "%.06f\n", camera_params[i]);
    }
    printf("\nDone writing to file.\n");
    fclose(fp);
}

int main(int argc, char **argv)
{
    char * correspondences_2d_file;
    char * correspondences_3d_file;
    char * output_file;
    /*location of observed facial landmarks*/
    double landmark_x_locs[NUM_LANDMARKS];
    double landmark_y_locs[NUM_LANDMARKS];
    /*3D correspondences*/
    double model_correspondences[NUM_LANDMARKS*3];
    /*camera*/
    double camera[NUM_CAMERA_PARAMS];
    /*ceres problem*/
    Problem problem;

    if (argc != 4)
    {
        printf("call as estimate-camera-params.cpp 2d 3d out_file\n");
        return 0;
    }
    correspondences_2d_file = argv[1];
    correspondences_3d_file = argv[2];
    output_file = argv[3];

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
    
    /*Convert estimated camera rotation parameters from rodriguez vector to roation matrix*/
    double rotation_matrix[9] = {0};
    get_rotation_matrix(camera, rotation_matrix);


    printf("Estimated camera matrix: \n\n");
    printf("Rotation: %f %f %f\n", camera[0], camera[1], camera[2]);
    printf("Translation: %f %f %f\n", camera[3], camera[4], camera[5]);
    printf("Focal length: %f\n", camera[6]);

    /*Print rotation matrix*/
    for (int i=0; i<3; i++)
    {
        printf("%.02f\t%.02f\t%.02f\n", rotation_matrix[3*i], rotation_matrix[3*i+1], rotation_matrix[3*i+2]);
    }
    
    write_estimated_parameters_to_file(output_file, camera, rotation_matrix);
    
    printf("\nDone!!\n");

    return 0;
}/*end main*/


