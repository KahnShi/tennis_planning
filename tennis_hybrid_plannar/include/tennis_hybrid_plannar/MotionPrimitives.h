#ifndef MOTION_PRIMITIVES_H_
#define MOTION_PRIMITIVES_H_

/* ros */
#include <ros/ros.h>
/* math */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

namespace motion_primitives
{
  class MotionPrimitives
  {
  public:
    MotionPrimitives();
    ~MotionPrimitives(){}

    /* Trajectory */
    int m_traj_order;
    double m_traj_period_time;
    Vector3d m_acc_t0;
    Vector3d m_acc_tn;
    Vector3d m_vel_t0;
    Vector3d m_vel_tn;
    Vector3d m_pos_t0;
    Vector3d m_pos_tn;
    VectorXd *m_traj_param_x_ptr;
    VectorXd *m_traj_param_y_ptr;
    VectorXd *m_traj_param_z_ptr;

    void inputTrajectoryParam(int traj_order, double period_time, Vector3d *state_t0, Vector3d *state_tn);
    Vector3d getTrajectoryPoint(double t, int dev_order = 0);
    double getTrajectoryJerkCost();
    inline int permutation(int n, int order);
    void printTrajectoryParamaters();
  };
}

#endif
