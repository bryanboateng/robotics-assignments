#include "cv_control.h"

#include <opencv2/core/types_c.h>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

// Parameters for Visual Servoing
float f;                   // Focal length in pixel
float img_width;           // width of image sensor in pixel
float img_height;          // height of image sensor in pixel
float diameter_real;       // real diameter of the circle
float diameter_desired_px; // desired diameter of the circle in pixels
float dt;                  // Time step
float t0;                  // how fast the velocity controller should converge
int hcd_min_distance;
PrVector3 desired_s_opencvf;

void initVisualServoing(
    float _f,
    float _img_width,
    float _img_height,
    float _diameter_real,
    float _diameter_desired_px,
    float _dt,
    PrVector3 _desired_s_opencvf
) {
  f = _f;
  img_width = _img_width;
  img_height = _img_height;
  diameter_real = _diameter_real;
  diameter_desired_px = _diameter_desired_px;
  dt = _dt;
  desired_s_opencvf = _desired_s_opencvf;
  t0 = 0.3; // time constant for controller convergence. Smaller values result
            // in higher velocities. Transforms an error into a velocity
  hcd_min_distance = 2000;
}

/*****************************************************************************************************************/
/* YOUR WORK STARTS HERE!!! */

/**
 * findCircleFeature
 * Find circles in the image using the OpenCV Hough Circle detector
 *
 * Input parameters:
 *  img: the camera image, you can print text in it with
 * 	    putText(img,"Hello
 *World",cvPoint(0,12),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,0,255)) see
 *http://opencv.willowgarage.com/documentation/cpp/drawing_functions.html#cv-puttext
 *
 *  backproject: grayscale image with high values where the color of the image
 *is like the selected color.
 *
 * Output:
 *  crcl: as a result of this function you should write the center and radius of
 *the detected circle into crcl
 */
bool findCircleFeature(Mat &img, Mat &backproject, Circle &crcl) {
  putText(
      img, "Hello World", cvPoint(0, 12), FONT_HERSHEY_SIMPLEX, 0.5,
      CV_RGB(0, 0, 255)
  );
  Mat blurred;
  medianBlur(backproject, blurred, 5);
  vector<Vec3f> circles;
  HoughCircles(
      blurred, circles, HOUGH_GRADIENT, 1, blurred.rows / 16, 100, 30, 0, 0
  );
  Vec3f circle;
  crcl.center = Point2d(circle[0], circle[1]);
  crcl.radius = circle[2];
  return true;
}

/**
 * getImageJacobianCFToFF
 * Compute the Image Jacobian to map from
 * camera velocity in Camera Frame to feature velocities in Feature Frame
 *
 * You should use getImageJacobianFFToCF in controlRobot
 *
 * Input parameters:
 *  u and v: the current center of the circle in feature frame [pixels]
 *  z: the estimated depth of the circle [meters]
 *  f: the focal length [pixels]
 *  diameter: the real diameter of the circle [meters]
 *
 * Output:
 *  Jv: assign your image 3x3 Jacobian.
 */
void getImageJacobianCFToFF(
    PrMatrix3 &Jv,
    float u,
    float v,
    float z,
    float f,
    float diameter
) {
  const float d = f * diameter / z;

  Jv[0][0] = -f / z;
  Jv[0][1] = 0;
  Jv[0][2] = u / z;

  Jv[1][0] = 0;
  Jv[1][1] = -f / z;
  Jv[1][2] = v / z;

  Jv[2][0] = 0;
  Jv[2][1] = 0;
  Jv[2][2] = d / z;
}

/**
 * estimateCircleDepth
 * Estimates and returns the depth of the circle
 *
 * Input parameters:
 *  f: the focal length [pixels]
 *  diameter: the real diameter of the circle [meters]
 *  crcl: the parameters of the detected circle in the image
 *
 * Output return:
 *  depth of the circle wrt the camera [meters]
 */
float estimateCircleDepth(float f, float diameter, Circle &crcl) {
  return diameter * f / (crcl.radius * 2);
}

/**
 * transformFromOpenCVFToFF
 * Transform a feature vector from openCV frame (origin in upper left corner of
 * the image) to feature frame (origin at the center of the image)
 *
 * Input parameter:
 *  vector_opencvf: feature vector defined in opencv frame
 *
 * Output:
 *  vector_ff: feature vector defined in feature frame
 */
void transformFromOpenCVFToFF(PrVector3 vector_opencvf, PrVector3 &vector_ff) {
  vector_ff[0] = vector_opencvf[0] - img_height / 2;
  vector_ff[1] = vector_opencvf[1] - img_width / 2;
  vector_ff[2] = vector_opencvf[2];
}

/**
 * transformVelocityFromCFToEEF
 * Transform the desired velocity vector from camera frame to end-effector frame
 * You can hard code this transformation according to the fixed transformation
 * between the camera and the end effector (see the sketch in your assignment)
 *
 * Input parameter:
 *  vector_cf: velocity vector defined in camera frame
 *
 * Output:
 *  vector_eef: velocity vector defined in end-effector frame
 */
void transformVelocityFromCFToEEF(PrVector3 vector_cf, PrVector3 &vector_eef) {
  vector_eef[0] = -vector_cf[1];
  vector_eef[1] = vector_cf[0];
  vector_eef[2] = vector_cf[2];
}

/**
 * transformVelocityFromEEFToBF
 * Transform the desired velocity vector from end-effector frame to base frame
 * You cannot hard code this transformation because it depends of the current
 * orientation of the end-effector wrt the base Make use of the current state of
 * the robot x (the pose of the end-effector in base frame coordinates)
 *
 * Input parameters:
 *  x_current_bf: current state of the robot - pose of the end-effector in base
 * frame coordinates vector_eef: velocity vector defined in end-effector frame
 *
 * Output:
 *  vector_bf: velocity vector defined in base frame
 */
void transformVelocityFromEEFToBF(
    PrVector x_current_bf,
    PrVector3 vector_eef,
    PrVector3 &vector_bf
) {
  PrMatrix3 rotation_matrix;

  double w = x_current_bf[3];
  double x = x_current_bf[4];
  double y = x_current_bf[5];
  double z = x_current_bf[6];

  rotation_matrix[0][0] = 1.0 - 2.0 * (y * y + z * z);
  rotation_matrix[0][1] = 2.0 * (x * y - w * z);
  rotation_matrix[0][2] = 2.0 * (x * z + w * y);

  rotation_matrix[1][0] = 2.0 * (x * y + w * z);
  rotation_matrix[1][1] = 1.0 - 2.0 * (x * x + z * z);
  rotation_matrix[1][2] = 2.0 * (y * z - w * x);

  rotation_matrix[2][0] = 2.0 * (x * z - w * y);
  rotation_matrix[2][1] = 2.0 * (y * z + w * x);
  rotation_matrix[2][2] = 1.0 - 2.0 * (x * x + y * y);
  vector_bf = rotation_matrix * vector_eef;
}

/*
 * controlRobot
 * This function computes the command to be send to the robot using Visual
 *Servoing so that the robot tracks the circle
 *
 * Here you should:
 * - compute the error in feature frame
 * - compute the circle depth
 * - compute the image jacobian from feature frame in camera frame
 * - compute the desired ee velocity in feature frame
 * - compute the desired ee velocity in camera frame
 * - compute the desired ee velocity in ee frame
 * - compute the desired ee velocity in base frame
 * - compute the step in the direction of the desired ee velocity in base frame
 * - form the comand to be sent to the robot (previous pose + computed step)
 *
 * The function will only be called if findCircleFeature returns true (if a
 *circle is detected in the image)
 *
 * Input parameters:
 *  crcl: the parameters of the detected circle in the image
 *  x:	current robot configuration in operational space (7 dof: 3 first values
 *are position, 4 last values is orientation quaternion) img: the camera image
 *for drawing debug text
 *
 * Output:
 *  cmdbuf: should contain the command for the robot controler, for example:
 *			"goto 0.0 0.0 90.0 0.0 0.0 0.0"
 */

void controlRobot(Circle &crcl, PrVector &x, Mat &img, char *cmdbuf) {
  if (crcl.radius == 0) {
    sprintf(cmdbuf, "float");
    return;
  }

  PrVector3 current_s_opencvf;
  current_s_opencvf[0] = crcl.center.x;
  current_s_opencvf[1] = crcl.center.y;
  current_s_opencvf[2] = 2 * crcl.radius;

  PrVector3 desired_s_ff;
  transformFromOpenCVFToFF(desired_s_opencvf, desired_s_ff);
  PrVector3 current_s_ff;
  transformFromOpenCVFToFF(current_s_opencvf, current_s_ff);

  PrVector3 error_s_ff = desired_s_ff - current_s_ff;

  float z = estimateCircleDepth(f, diameter_real, crcl);

  PrMatrix3 Jv;
  getImageJacobianCFToFF(
      Jv, current_s_ff[0], current_s_ff[1], z, f, diameter_real
  );

  PrMatrix3 Jv_inv;
  Jv.pseudoInverse(Jv_inv);

  // Compute the desired velocity of the feature in feature frame
  PrVector3 vel_f_ff = error_s_ff / t0;

  // Compute the desired velocity of the end effector in camera frame
  PrVector3 vel_ee_cf = Jv_inv * vel_f_ff;

  PrVector3 vel_ee_eef;
  transformVelocityFromCFToEEF(vel_ee_cf, vel_ee_eef);

  PrVector3 vel_ee_bf;
  transformVelocityFromEEFToBF(x, vel_ee_eef, vel_ee_bf);

  // compute the next EE position for the next timestep given the desired EE
  // velocity:
  PrVector3 step_ee_bf = vel_ee_bf * dt;

  PrVector desired_ee_pose_bf = x;
  desired_ee_pose_bf[0] += step_ee_bf[0];
  desired_ee_pose_bf[1] += step_ee_bf[1];
  desired_ee_pose_bf[2] += step_ee_bf[2];

  // Command the robot to go to the new desired position:
  sprintf(
      cmdbuf, "goto %.4f %.4f %.4f %.4f %.4f %.4f %.4f", desired_ee_pose_bf[0],
      desired_ee_pose_bf[1], desired_ee_pose_bf[2], 0.50, 0.50, -0.50, 0.50
  );

  putText(
      img, cmdbuf, cv::Point(5, 50), FONT_HERSHEY_SIMPLEX, 0.3,
      CV_RGB(0, 255, 0), 1.2
  );
}
