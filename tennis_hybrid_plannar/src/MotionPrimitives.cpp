#include <tennis_hybrid_plannar/MotionPrimitives.h>

namespace motion_primitives{
  MotionPrimitives::MotionPrimitives()
  {
    /* Give a random initialization for paramater ptr */
    m_traj_param_x_ptr = new VectorXd(1);
    m_traj_param_y_ptr = new VectorXd(1);
    m_traj_param_z_ptr = new VectorXd(1);
  }
  void MotionPrimitives::inputTrajectoryParam(int traj_order, double period_time, Vector3d *state_t0, Vector3d *state_tn)
  {
    m_traj_order = traj_order;
    delete m_traj_param_x_ptr;
    delete m_traj_param_y_ptr;
    delete m_traj_param_z_ptr;
    m_traj_param_x_ptr = new VectorXd(m_traj_order + 1);
    m_traj_param_y_ptr = new VectorXd(m_traj_order + 1);
    m_traj_param_z_ptr = new VectorXd(m_traj_order + 1);

    m_traj_period_time = period_time;
    m_pos_t0 = state_t0[0];
    m_vel_t0 = state_t0[1];
    m_acc_t0 = state_t0[2];
    m_pos_tn = state_tn[0];
    m_vel_tn = state_tn[1];
    m_acc_tn = state_tn[2];

    MatrixXd mat = MatrixXd::Zero(m_traj_order-2, 3);
    if (m_traj_order == 5){
      mat(0, 0) = 60 * pow(m_traj_period_time, 2);
      mat(0, 1) = -360 * m_traj_period_time;
      mat(0, 2) = 720;
      mat(1, 0) = -24 * pow(m_traj_period_time, 3);
      mat(1, 1) = 168 * pow(m_traj_period_time, 2);
      mat(1, 2) = -360 * m_traj_period_time;
      mat(2, 0) = 3 * pow(m_traj_period_time, 4);
      mat(2, 1) = -24 * pow(m_traj_period_time, 3);
      mat(2, 2) = 60 * pow(m_traj_period_time, 2);

      Vector3d d_state, param;
      /* get x param */
      d_state[0] = m_acc_tn[0] - m_acc_t0[0];
      d_state[1] = m_vel_tn[0] - m_vel_t0[0] - m_acc_t0[0] * m_traj_period_time;
      d_state[2] = m_pos_tn[0] - m_pos_t0[0] - m_vel_t0[0] * m_traj_period_time - 0.5 * m_acc_t0[0] * pow(m_traj_period_time, 2);
      param = mat * d_state / pow(m_traj_period_time, 5);
      (*m_traj_param_x_ptr)[5] = param[0] / 120.0;
      (*m_traj_param_x_ptr)[4] = param[1] / 24.0;
      (*m_traj_param_x_ptr)[3] = param[2] / 6.0;
      (*m_traj_param_x_ptr)[2] = m_acc_t0[0] / 2.0;
      (*m_traj_param_x_ptr)[1] = m_vel_t0[0];
      (*m_traj_param_x_ptr)[0] = m_pos_t0[0];
      /* get y param */
      d_state[0] = m_acc_tn[1] - m_acc_t0[1];
      d_state[1] = m_vel_tn[1] - m_vel_t0[1] - m_acc_t0[1] * m_traj_period_time;
      d_state[2] = m_pos_tn[1] - m_pos_t0[1] - m_vel_t0[1] * m_traj_period_time - 0.5 * m_acc_t0[1] * pow(m_traj_period_time, 2);
      param = mat * d_state / pow(m_traj_period_time, 5);
      (*m_traj_param_y_ptr)[5] = param[0] / 120.0;
      (*m_traj_param_y_ptr)[4] = param[1] / 24.0;
      (*m_traj_param_y_ptr)[3] = param[2] / 6.0;
      (*m_traj_param_y_ptr)[2] = m_acc_t0[1] / 2.0;
      (*m_traj_param_y_ptr)[1] = m_vel_t0[1];
      (*m_traj_param_y_ptr)[0] = m_pos_t0[1];
      /* get z param */
      d_state[0] = m_acc_tn[2] - m_acc_t0[2];
      d_state[1] = m_vel_tn[2] - m_vel_t0[2] - m_acc_t0[2] * m_traj_period_time;
      d_state[2] = m_pos_tn[2] - m_pos_t0[2] - m_vel_t0[2] * m_traj_period_time - 0.5 * m_acc_t0[2] * pow(m_traj_period_time, 2);
      param = mat * d_state / pow(m_traj_period_time, 5);
      (*m_traj_param_z_ptr)[5] = param[0] / 120.0;
      (*m_traj_param_z_ptr)[4] = param[1] / 24.0;
      (*m_traj_param_z_ptr)[3] = param[2] / 6.0;
      (*m_traj_param_z_ptr)[2] = m_acc_t0[2] / 2.0;
      (*m_traj_param_z_ptr)[1] = m_vel_t0[2];
      (*m_traj_param_z_ptr)[0] = m_pos_t0[2];
    }
    else{
      ROS_ERROR("[MotionPrimitives] Currently we only support 5-th order trajectory.");
    }
  }

  Vector3d MotionPrimitives::getTrajectoryPoint(double t, int dev_order)
  {
    if (t > m_traj_period_time){
      ROS_WARN("[MotionPrimitives] getTrajectoryPoint time is larger than period_time, %f", t);
      t = m_traj_period_time;
    }
    Vector3d value(0.0, 0.0, 0.0);
    for (int i = dev_order; i <= m_traj_order; ++i){
      value[0] += (*m_traj_param_x_ptr)[i] * pow(t, i - dev_order) * permutation(i, dev_order);
      value[1] += (*m_traj_param_y_ptr)[i] * pow(t, i - dev_order) * permutation(i, dev_order);
      value[2] += (*m_traj_param_z_ptr)[i] * pow(t, i - dev_order) * permutation(i, dev_order);
    }
    return value;
  }

  inline int MotionPrimitives::permutation(int n, int order)
  {
    int result = 1;
    for (int i = 0; i < order; ++i)
      result *= (n - i);
    return result;
  }

  double MotionPrimitives::getTrajectoryJerkCost()
  {
    double costs = 0.0;
    for (int i = 3; i <= m_traj_order; ++i)
      for (int j = 3; j <= m_traj_order; ++j){
        costs += permutation(i, 3) * permutation(j, 3) * pow(m_traj_period_time, i+j-5) / (i+j-5)
          * ((*m_traj_param_x_ptr)[i] * (*m_traj_param_x_ptr)[j]
             + (*m_traj_param_y_ptr)[i] * (*m_traj_param_y_ptr)[j]
             + (*m_traj_param_z_ptr)[i] * (*m_traj_param_z_ptr)[j]);
      }
    costs /= m_traj_period_time;
    return costs;
  }
}
