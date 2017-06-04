#include <tennis_hybrid_plannar/SnakeCommand.h>

namespace snake_command{
  SnakeCommand::SnakeCommand(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("pub_flight_nav_topic_name", m_pub_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("pub_joints_ctrl_topic_name", m_pub_joints_ctrl_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("sub_move_start_flag_topic_name", m_sub_move_start_flag_topic_name, std::string("/move_start"));
    m_nhp.param("sub_joint_states_topic_name", m_sub_joint_states_topic_name, std::string("/hydrus3/joint_states"));
    m_nhp.param("sub_base_link_odom_topic_name", m_sub_base_link_odom_topic_name, std::string("/uav/state"));
    m_nhp.param("snake_links_number", m_n_links, 4);
    m_nhp.param("snake_link_length", m_link_length, 0.44);
    m_nhp.param("control_period", m_control_period, 0.05);

    m_move_start_flag = false;
    m_links_pos_ptr = new tf::Vector3[m_n_links + 1];
    m_links_vel_ptr = new tf::Vector3[m_n_links + 1];
    m_joints_ang_ptr = new double[m_n_links + 1];
    m_joints_ang_vel_ptr = new double[m_n_links + 1];

    // /hydrus3/joint_states get joints angle and angle vel
    // /uav/state get uav link2's global position, velocity, orientation
    m_pub_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_flight_nav_topic_name, 1);
    m_pub_joints_ctrl = m_nh.advertise<sensor_msgs::JointState>(m_pub_joints_ctrl_topic_name, 1);

    m_sub_move_start_flag = m_nh.subscribe<std_msgs::Empty>(m_sub_move_start_flag_topic_name, 1, &SnakeCommand::moveStartFlagCallback, this);
    m_sub_joint_states = m_nh.subscribe<sensor_msgs::JointState>(m_sub_joint_states_topic_name, 1, &SnakeCommand::jointStatesCallback, this);
    m_sub_base_link_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_base_link_odom_topic_name, 1, &SnakeCommand::baseLinkOdomCallback, this);
    m_timer = m_nh.createTimer(ros::Duration(m_control_period), &SnakeCommand::controlCallback, this);

    m_traj_start_time = -1.0;
    m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
  }

  void SnakeCommand::controlCallback(const ros::TimerEvent& e)
  {
    if (!m_move_start_flag)
      return;
    if (m_traj_start_time < 0)
      m_traj_start_time = e.current_real.toSec();

    m_traj_current_time = e.current_real.toSec();

    directTrackGlobalTrajectory();
  }

  void SnakeCommand::directTrackGlobalTrajectory()
  {
    double current_traj_time = (m_traj_current_time - m_traj_start_time);
    if (current_traj_time >= m_traj_primitive->m_traj_period_time){
      if (current_traj_time - m_traj_primitive->m_traj_period_time < 0.1)
        ROS_INFO("\nArrived at last control point. \n");
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.header.frame_id = std::string("/world");
      nav_msg.header.stamp = ros::Time::now();
      nav_msg.header.seq = 1;
      nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
      nav_msg.target_vel_x = 0.0;
      nav_msg.target_vel_y = 0.0;
      nav_msg.psi_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_psi = m_traj_fixed_yaw;
      m_pub_flight_nav.publish(nav_msg);
      return;
    }
    tf::Vector3 des_world_vel = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(current_traj_time, 1));
    tf::Vector3 des_world_pos = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(current_traj_time, 0));
    tf::Vector3 real_world_pos;
    // link1
    real_world_pos = m_racket_1_pos;

    /* pid control in trajectory tracking */
    tf::Vector3 traj_track_p_term =  (des_world_pos - real_world_pos) * 0.45;
    m_traj_track_i_term_accumulation += (des_world_pos - real_world_pos) * m_control_period;
    tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * 0.0 * 0.1;

    /* feedforward */
    tf::Vector3 vel = des_world_vel + traj_track_p_term + traj_track_i_term;

    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
    nav_msg.target_vel_x = vel.getX();
    nav_msg.target_vel_y = vel.getY();
    nav_msg.psi_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_psi = m_traj_fixed_yaw;
    m_pub_flight_nav.publish(nav_msg);
  }

  void SnakeCommand::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg)
  {
    for (int i = 0; i < joints_msg->position.size(); ++i){
      m_joints_ang_ptr[i+1] = joints_msg->position[i];
      m_joints_ang_vel_ptr[i+1] = joints_msg->velocity[i];
    }
    /* calculate every links data from base_link */
    for (int i = 1; i <= 5; ++i){
      tf::StampedTransform transform;
      geometry_msgs::Twist link_twist;
      m_tf_listener.lookupTransform("/world", "/link" + std::to_string(i), ros::Time(0), transform);
      m_tf_listener.lookupTwist("/world", "/link" + std::to_string(i), ros::Time(0), ros::Duration(0.1), link_twist);
      m_links_pos_ptr[i-1].setValue(transform.getOrigin().x(),
                                    transform.getOrigin().y(),
                                    transform.getOrigin().z());
      m_links_vel_ptr[i-1].setValue(link_twist.linear.x,
                                    link_twist.linear.y,
                                    link_twist.linear.z);
      // std::cout << "/link" + std::to_string(i) << ": " << link_twist.linear.x << ", "
      //           << link_twist.linear.y << ", " << link_twist.linear.z << "\n";
    }
    tf::StampedTransform transform;
    m_tf_listener.lookupTransform("/world", "/link_racket_1_center", ros::Time(0), transform);
    m_racket_1_pos.setValue(transform.getOrigin().x(),
                             transform.getOrigin().y(),
                             transform.getOrigin().z());
  }

  void SnakeCommand::baseLinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    m_base_link_odom = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    m_base_link_ang.setValue(r, p, y);
    m_base_link_pos.setValue(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
    m_base_link_vel.setValue(odom_msg->twist.twist.linear.x,
                             odom_msg->twist.twist.linear.y,
                             odom_msg->twist.twist.linear.z);
  }

  void SnakeCommand::moveStartFlagCallback(const std_msgs::Empty msg)
  {
    m_move_start_flag = true;
  }

  inline tf::Vector3 SnakeCommand::vectorToVector3(std::vector<double> vec)
  {
    tf::Vector3 vec3(vec[0], vec[1], vec[2]);
    return vec3;
  }

  inline tf::Vector3 SnakeCommand::vector3dToVector3(Vector3d vec)
  {
    tf::Vector3 vec3(vec[0], vec[1], vec[2]);
    return vec3;
  }

}
