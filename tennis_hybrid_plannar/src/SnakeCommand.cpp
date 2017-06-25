#include <tennis_hybrid_plannar/SnakeCommand.h>

namespace snake_command{
  SnakeCommand::SnakeCommand(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("pub_flight_nav_topic_name", m_pub_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("pub_joints_ctrl_topic_name", m_pub_joints_ctrl_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("sub_move_start_flag_topic_name", m_sub_move_start_flag_topic_name, std::string("/move_start"));
    m_nhp.param("sub_joint_states_topic_name", m_sub_joint_states_topic_name, std::string("/hydrus3/joint_states"));
    m_nhp.param("sub_base_link_odom_topic_name", m_sub_base_link_odom_topic_name, std::string("/uav/root_link/odom"));
    m_nhp.param("snake_links_number", m_n_links, 4);
    m_nhp.param("snake_link_length", m_link_length, 0.44);
    m_nhp.param("control_period", m_control_period, 0.05);
    m_nhp.param("snake_acceleration_max_value", m_snake_acceleration_max_value, 0.3);

    m_move_start_flag = false;
    m_links_pos_ptr = new tf::Vector3[m_n_links + 1];
    m_links_vel_ptr = new tf::Vector3[m_n_links + 1];
    m_joints_ang_ptr = new double[m_n_links + 1];
    m_joints_ang_vel_ptr = new double[m_n_links + 1];

    // /hydrus3/joint_states get joints angle and angle vel
    // /uav/state get uav link2's global position, velocity, orientation
    m_pub_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_flight_nav_topic_name, 1);
    m_pub_joints_ctrl = m_nh.advertise<sensor_msgs::JointState>(m_pub_joints_ctrl_topic_name, 1);
    m_pub_racket_center_expected_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/racket_center_expected_markers", 1);
    // todo: cheat mode
    m_pub_ball_start_odom = m_nh.advertise<nav_msgs::Odometry>("/gazebo_ping_pong_ball_start_odom_1", 1);

    m_sub_move_start_flag = m_nh.subscribe<std_msgs::Empty>(m_sub_move_start_flag_topic_name, 1, &SnakeCommand::moveStartFlagCallback, this);
    m_sub_joint_states = m_nh.subscribe<sensor_msgs::JointState>(m_sub_joint_states_topic_name, 1, &SnakeCommand::jointStatesCallback, this);
    m_sub_base_link_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_base_link_odom_topic_name, 1, &SnakeCommand::baseLinkOdomCallback, this);
    m_sub_cog_world_coord = m_nh.subscribe<aerial_robot_base::DesireCoord>(std::string("/desire_coordinate"), 1, &SnakeCommand::cogWorldCoordCallback, this);
    m_timer = m_nh.createTimer(ros::Duration(m_control_period), &SnakeCommand::controlCallback, this);

    initializeVariable();
  }

  void SnakeCommand::initializeVariable()
  {
    m_move_start_flag = false;
    m_traj_start_time = -1.0;
    m_traj_track_i_term_accumulation.setValue(0.0, 0.0, 0.0);
    m_traj_track_state = TRAJ_TRACK_NOT_START;
    m_racket_state = RACKET_RELEX;
  }

  void SnakeCommand::controlCallback(const ros::TimerEvent& e)
  {
    if (!m_move_start_flag)
      return;
    /* first time to get control robot */
    if (m_traj_start_time < 0){
      m_traj_start_time = e.current_real.toSec();
      // todo: currently give offset in z axis, in case delay of position control
      // but future need to change to pid based velocity control
      m_traj_control_z_offset = (m_racket_1_pos.getZ() - m_traj_primitive->m_pos_tn[2]) * 0.125;
      if (m_traj_control_z_offset > 0.1)
        m_traj_control_z_offset = 0.1;
      m_traj_track_state = TRAJ_TRACK_ON_GOING;
      m_racket_1_base_link_offset = m_links_pos_ptr[1] - m_racket_1_pos;

      // todo : throw ball cheat mode
      if (!m_tennis_ball_static_hit_mode){
        nav_msgs::Odometry ball_start_odom;
        ball_start_odom.pose.pose.orientation.x = 0.0;
        ball_start_odom.pose.pose.orientation.y = 0.0;
        ball_start_odom.pose.pose.orientation.z = 0.0;
        ball_start_odom.pose.pose.orientation.w = 1.0;
        tf::Vector3 ball_start_vel(0.0, -3.0, 9.81 * m_traj_primitive->m_traj_period_time / 2.0);
        tf::Vector3 ball_start_pos = vector3dToVector3(m_traj_primitive->m_pos_tn) - ball_start_vel * m_traj_primitive->m_traj_period_time;
        ball_start_pos.setZ(m_racket_1_pos.getZ() + 9.81 / 2 * pow(m_traj_primitive->m_traj_period_time, 2.0) - ball_start_vel.getZ() * m_traj_primitive->m_traj_period_time);

        ball_start_odom.pose.pose.position.x = ball_start_pos.getX();
        ball_start_odom.pose.pose.position.y = ball_start_pos.getY();
        ball_start_odom.pose.pose.position.z = ball_start_pos.getZ();
        ball_start_odom.twist.twist.linear.x = ball_start_vel.getX();
        ball_start_odom.twist.twist.linear.y = ball_start_vel.getY();
        ball_start_odom.twist.twist.linear.z = ball_start_vel.getZ();
        m_pub_ball_start_odom.publish(ball_start_odom);
      }
    }

    m_traj_current_time = e.current_real.toSec();
    directTrackGlobalTrajectory();
  }

  void SnakeCommand::directTrackGlobalTrajectory()
  {
    double current_traj_time = (m_traj_current_time - m_traj_start_time);
    double sanke_joint_vel = 0.785;
    double mininum_racket_compress_start_time = 0.5;
    double snake_joint_compress_ang = -(m_traj_primitive->m_traj_period_time -
                                        mininum_racket_compress_start_time)
      / 2.0 * sanke_joint_vel;
    // wave angle do not exceed pi/4
    if (snake_joint_compress_ang < -0.785)
      snake_joint_compress_ang = -0.785;
    double racket_compress_time = fabs(snake_joint_compress_ang) / sanke_joint_vel;
    if (m_traj_track_state == TRAJ_TRACK_FINISH || current_traj_time >= m_traj_primitive->m_traj_period_time){
      if (m_traj_track_state == TRAJ_TRACK_ON_GOING){
        m_traj_track_state = TRAJ_TRACK_FINISH;
        tf::Vector3 des_world_pos = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(m_traj_primitive->m_traj_period_time, 0))
          + m_racket_1_base_link_offset;
        m_traj_finish_height = des_world_pos.getZ();
        /* if finish height is too low, robot will crash into table */
        // todo: several states before reaching to 2.0m. eg. first 1.5m, then 2.0m
        // otherwise there will be a positive speed in z axis at finishing point, because trajectory time is not strictly correspond to uav hitting time
        if (m_traj_finish_height < 2.0)
          m_traj_finish_height = 2.0;
      }
      aerial_robot_base::FlightNav nav_msg;
      nav_msg.header.frame_id = std::string("/world");
      nav_msg.header.stamp = ros::Time::now();
      nav_msg.header.seq = 1;
      nav_msg.pos_xy_nav_mode = nav_msg.ACC_MODE;
      // todo: set suitable command when finish the trajectory
      tf::Vector3 cur_vel(m_base_link_vel.getX(), m_base_link_vel.getY(), m_base_link_vel.getZ());
      tf::Vector3 acc = -cur_vel * 1.0;
      acc.setMax(tf::Vector3(-m_snake_acceleration_max_value, -m_snake_acceleration_max_value, -m_snake_acceleration_max_value));
      acc.setMin(tf::Vector3(m_snake_acceleration_max_value, m_snake_acceleration_max_value, m_snake_acceleration_max_value));
      nav_msg.target_acc_x = acc.getX();
      nav_msg.target_acc_y = acc.getY();
      nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
      nav_msg.target_pos_z = m_traj_finish_height;
      m_pub_flight_nav.publish(nav_msg);
      return;
    }
    if (m_racket_state == RACKET_COMPRESS
        && current_traj_time >= m_traj_primitive->m_traj_period_time - racket_compress_time){
      m_racket_state = RACKET_HIT;
      /* wave racket */
      // here racket is static
      sensor_msgs::JointState joints_msg;
      joints_msg.position.push_back(1.5708);
      joints_msg.position.push_back(1.5708);
      joints_msg.position.push_back(0.0);
      m_pub_joints_ctrl.publish(joints_msg);
    }
    else if (m_racket_state == RACKET_RELEX
             && m_traj_primitive->m_traj_period_time >= racket_compress_time * 2
             && current_traj_time >= m_traj_primitive->m_traj_period_time - racket_compress_time * 2){
      m_racket_state = RACKET_COMPRESS;
      sensor_msgs::JointState joints_msg;
      joints_msg.position.push_back(snake_joint_compress_ang);
      joints_msg.position.push_back(1.5708);
      joints_msg.position.push_back(0.0);
      m_pub_joints_ctrl.publish(joints_msg);
    }
    tf::Vector3 des_world_vel = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(current_traj_time, 1));
    tf::Vector3 des_world_pos = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(current_traj_time, 0)) + m_racket_1_base_link_offset;
    /* Visualization for expected racket position */
    visualizeRacketExpectedPosition(des_world_pos - m_racket_1_base_link_offset);
    tf::Vector3 real_world_pos;
    real_world_pos = m_links_pos_ptr[1];

    /* pid control in trajectory tracking */
    tf::Vector3 traj_track_p_term =  (des_world_pos - real_world_pos) * 0.5;
    m_traj_track_i_term_accumulation += (des_world_pos - real_world_pos) * m_control_period;
    tf::Vector3 traj_track_i_term = m_traj_track_i_term_accumulation * 0.1;

    /* feedforward */
    tf::Vector3 des_vel = des_world_vel + traj_track_p_term + traj_track_i_term;
    tf::Vector3 real_vel(m_base_link_vel.getX(), m_base_link_vel.getY(), m_base_link_vel.getZ());
    tf::Vector3 des_acc = vector3dToVector3(m_traj_primitive->getTrajectoryPoint(current_traj_time, 2));
    tf::Vector3 acc = (des_acc + (des_vel - real_vel) * 0.5);

    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 1;
    nav_msg.pos_xy_nav_mode = nav_msg.ACC_MODE;
    acc.setMax(tf::Vector3(-m_snake_acceleration_max_value, -m_snake_acceleration_max_value, -m_snake_acceleration_max_value));
    acc.setMin(tf::Vector3(m_snake_acceleration_max_value, m_snake_acceleration_max_value, m_snake_acceleration_max_value));
    nav_msg.target_acc_x = acc.getX();
    nav_msg.target_acc_y = acc.getY();
    nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_z = m_traj_finish_height;
    m_pub_flight_nav.publish(nav_msg);
    nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_z = des_world_pos.getZ() - m_traj_control_z_offset;
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
    m_tf_listener.lookupTransform("/world", "/link_racket_2_center", ros::Time(0), transform);
    m_racket_2_pos.setValue(transform.getOrigin().x(),
                            transform.getOrigin().y(),
                            transform.getOrigin().z());
    // change offset because roll pitch tilt angle will influence it.
    if (m_move_start_flag){
      // method 1, only adjust z axis every time
      // m_racket_1_base_link_offset.setZ(m_links_pos_ptr[1].getZ() - m_racket_1_pos.getZ());
      // method 2
      m_racket_1_base_link_offset = m_links_pos_ptr[1] - m_racket_1_pos;
    }
  }

  tf::Vector3 SnakeCommand::attitudeCvtWorldToCog(tf::Vector3 att_world)
  {
    tf::StampedTransform transform;
    m_tf_listener.lookupTransform("/link2", "/world", ros::Time(0), transform);
    tf::Matrix3x3 mat_l2_w = transform.getBasis();
    double r_l2_w, p_l2_w, y_l2_w;
    mat_l2_w.getRPY(r_l2_w, p_l2_w, y_l2_w);
    tf::Matrix3x3 mat_l2_w_flat, mat_cog_l2_flat;
    mat_l2_w_flat.setRPY(0, 0, y_l2_w + 1.5708);
    mat_cog_l2_flat.setRPY(0, 0, m_cog_world_rpy[2]);
    tf::StampedTransform tran_l2_w_flat, tran_cog_l2_flat;
    tran_l2_w_flat.setBasis(mat_l2_w_flat);
    tran_cog_l2_flat.setBasis(mat_cog_l2_flat);
    tf::Vector3 att_cog = tran_cog_l2_flat * (tran_l2_w_flat * att_world);
    return att_cog;
  }

  void SnakeCommand::cogWorldCoordCallback(const aerial_robot_base::DesireCoordConstPtr& coord_msg)
  {
    m_cog_world_rpy[0] = coord_msg->roll;
    m_cog_world_rpy[1] = coord_msg->pitch;
    m_cog_world_rpy[2] = coord_msg->yaw;
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
    double r,p,y;
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

  void SnakeCommand::visualizeRacketExpectedPosition(tf::Vector3 des_pos)
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker racket_center_expected_marker;
    racket_center_expected_marker.ns = "racket_center_expected";
    racket_center_expected_marker.header.frame_id = std::string("/world");
    racket_center_expected_marker.header.stamp = ros::Time().now();
    racket_center_expected_marker.action = visualization_msgs::Marker::ADD;
    racket_center_expected_marker.type = visualization_msgs::Marker::SPHERE;
    racket_center_expected_marker.pose.position.x = des_pos.getX();
    racket_center_expected_marker.pose.position.y = des_pos.getY();
    racket_center_expected_marker.pose.position.z = des_pos.getZ();
    racket_center_expected_marker.lifetime = ros::Duration(0.1);

    racket_center_expected_marker.scale.x = 0.2;
    racket_center_expected_marker.scale.y = 0.2;
    racket_center_expected_marker.scale.z = 0.2;
    racket_center_expected_marker.color.a = 0.5;
    racket_center_expected_marker.color.r = 1.0f;
    racket_center_expected_marker.color.g = 1.0f;
    racket_center_expected_marker.color.b = 0.0f;
    markers.markers.push_back(racket_center_expected_marker);

    m_pub_racket_center_expected_markers.publish(markers);
  }
}
