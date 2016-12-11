#ifndef CALCULATE_REPROJECTION_ERROR_H_
#define CALCULATE_REPROJECTION_ERROR_H_

#include <ceres/rotation.h>

struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = (focal * xp)+camera[7];
    T predicted_y = (focal * yp)+camera[8];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double observed_x,
                                      const double observed_y) {
     return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                 new SnavelyReprojectionError(observed_x, observed_y)));
   }

  double observed_x;
  double observed_y;
};


#endif /*CALCULATE_REPROJECTION_ERROR_H*/

