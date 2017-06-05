#include <tennis_hybrid_plannar/HybridPlannar.h>

namespace hybrid_plannar
{
  HybridPlannar::HybridPlannar(ros::NodeHandle nh, ros::NodeHandle nhp): m_nh(nh), m_nhp(nhp)
  {
    m_nhp.param("sub_snake_odom_topic_name", m_sub_snake_odom_topic_name, std::string("/uav/state"));
    m_nhp.param("sub_tennis_ball_odom_topic_name", m_sub_tennis_ball_odom_topic_name, std::string("/gazebo_ping_pong_ball_odom_1"));
    m_nhp.param("pub_snake_start_flag", m_pub_snake_start_flag_topic_name, std::string("/teleop_command/start"));
    m_nhp.param("pub_snake_takeoff_flag", m_pub_snake_takeoff_flag_topic_name, std::string("/teleop_command/takeoff"));
    m_nhp.param("pub_snake_land_flag", m_pub_snake_land_flag_topic_name, std::string("/teleop_command/land"));
    m_nhp.param("pub_snake_joint_states_topic_name", m_pub_snake_joint_states_topic_name, std::string("/hydrus3/joints_ctrl"));
    m_nhp.param("pub_snake_flight_nav_topic_name", m_pub_snake_flight_nav_topic_name, std::string("/uav/nav"));
    m_nhp.param("pub_snake_traj_path_topic_name", m_pub_snake_traj_path_topic_name, std::string("/traj_path"));
    m_nhp.param("snake_traj_order", m_snake_traj_order, 10);
    m_nhp.param("snake_links_number", m_n_snake_links, 4);
    m_nhp.param("snake_link_length", m_snake_link_length, 0.44);
    m_nhp.param("snake_average_vel", m_snake_average_vel, 0.5);

    /* subscriber & publisher */
    m_sub_snake_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_snake_odom_topic_name, 1, &HybridPlannar::snakeOdomCallback, this);
    m_sub_tennis_ball_odom = m_nh.subscribe<nav_msgs::Odometry>(m_sub_tennis_ball_odom_topic_name, 1, &HybridPlannar::tennisBallOdomCallback, this);
    m_sub_snake_task_start_flag = m_nh.subscribe<std_msgs::Empty>(std::string("/task_start"), 1, &HybridPlannar::taskStartCallback, this);

    m_pub_snake_start_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_start_flag_topic_name, 1);
    m_pub_snake_takeoff_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_takeoff_flag_topic_name, 1);
    m_pub_snake_land_flag = m_nh.advertise<std_msgs::Empty>(m_pub_snake_land_flag_topic_name, 1);
    m_pub_snake_joint_states = m_nh.advertise<sensor_msgs::JointState>(m_pub_snake_joint_states_topic_name, 1);
    m_pub_snake_flight_nav = m_nh.advertise<aerial_robot_base::FlightNav>(m_pub_snake_flight_nav_topic_name, 1);
    m_pub_tennis_ball_markers = m_nh.advertise<visualization_msgs::MarkerArray>("/tennis_ball_markers", 1);
    m_pub_snake_traj_path = m_nh.advertise<nav_msgs::Path>(m_pub_snake_traj_path_topic_name, 1);

    /* Init value */
    m_task_start_flag = false;
    m_snake_joint_states_vel_ptr = new double[m_n_snake_links + 1];
    m_snake_joint_states_ang_ptr = new double[m_n_snake_links + 1];
    m_snake_command_ptr = new SnakeCommand(m_nh, m_nhp);
    m_snake_command_ptr->m_traj_fixed_yaw = -0.785;
    m_traj_primitive_ptr = new MotionPrimitives();

    usleep(2000000);
    ROS_INFO("[HybridPlannar] Initialization finished.");

    snakeInitPose();
  }

  void HybridPlannar::snakeInitPose()
  {
    std_msgs::Empty msg;
    m_pub_snake_start_flag.publish(msg);
    usleep(300000);
    m_pub_snake_takeoff_flag.publish(msg);
    usleep(7000000);
    ROS_INFO("[HybridPlannar] Snake takeoff finished.");
    aerial_robot_base::FlightNav nav_yaw_msg;
    nav_yaw_msg.header.frame_id = std::string("/world");
    nav_yaw_msg.header.stamp = ros::Time::now();
    nav_yaw_msg.header.seq = 1;
    nav_yaw_msg.psi_nav_mode = nav_yaw_msg.POS_MODE;
    nav_yaw_msg.target_psi = -0.785;
    m_pub_snake_flight_nav.publish(nav_yaw_msg);
    usleep(5000000);
    sensor_msgs::JointState joints_msg;
    joints_msg.position.push_back(0.0);
    joints_msg.position.push_back(1.5708);
    joints_msg.position.push_back(0.0);
    m_pub_snake_joint_states.publish(joints_msg);
    usleep(5000000);
    // rotate to inital angle after transform
    nav_yaw_msg.header.seq = 2;
    m_pub_snake_flight_nav.publish(nav_yaw_msg);
    usleep(5000000);
    /* Fly to initial position */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.frame_id = std::string("/world");
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.header.seq = 3;
    nav_msg.pos_xy_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_x = -0.6;
    nav_msg.target_pos_y = -6.0;
    nav_msg.pos_z_nav_mode = nav_msg.POS_MODE;
    nav_msg.target_pos_z = 1.2;
    m_pub_snake_flight_nav.publish(nav_msg);
    usleep(5000000);
    /* Change yaw angle to initial value */
    /* Adjust to initial position again*/
    m_pub_snake_flight_nav.publish(nav_msg);
    usleep(2000000);
    ROS_INFO("[HybridPlannar] Snake reach initial joints state.");
  }

  void HybridPlannar::taskStartCallback(std_msgs::Empty msg)
  {
    m_task_start_flag = true;
    ROS_INFO("[HybridPlannar] Start to calculate trajectory from motion primitives.");
    // todo: add visualization flag
    if (getFesibleTrajectory()){
      visualizeTrajectory();
      m_snake_command_ptr->m_traj_primitive = m_traj_primitive_ptr;
    }
  }

  bool HybridPlannar::getFesibleTrajectory()
  {
    // todo: travel time should not be pre-set, but discrete
    double period_time;
    Vector3d d_dist(m_tennis_racket_1_odom.pose.pose.position.x - m_tennis_ball_odom.pose.pose.position.x,
                    m_tennis_racket_1_odom.pose.pose.position.y - m_tennis_ball_odom.pose.pose.position.y,
                    m_tennis_racket_1_odom.pose.pose.position.z - m_tennis_ball_odom.pose.pose.position.z);
    double avg_vel = 1.0;
    period_time = sqrt(pow(d_dist[0], 2) + pow(d_dist[1], 2) + pow(d_dist[2], 2)) / avg_vel;

    std::cout << "[HybridPlannar] Period time: " << period_time << "\n\n";

    // discrete velocity [0, 2.0], resolution 0.2
    // discrete acceleration [-2.0, 2.0], resolution 0.4
    double vel_min = 0.0, vel_max = 2.0, acc_min = -2.0, acc_max = 2.0;
    double vel_res = 0.2*2, acc_res = 0.4*2;
    double min_cost = -1;
    int vel_cnts = int((vel_max-vel_min)/vel_res), acc_cnts = int((acc_max-acc_min)/acc_res);
    for (int vel_x_id = 0; vel_x_id <= vel_cnts; ++vel_x_id)
      for (int vel_y_id = 0; vel_y_id <= vel_cnts; ++vel_y_id)
        for (int vel_z_id = 0; vel_z_id <= vel_cnts; ++vel_z_id)
          for (int acc_x_id = 0; acc_x_id <= acc_cnts; ++acc_x_id)
            for (int acc_y_id = 0; acc_y_id <= acc_cnts; ++acc_y_id)
              for (int acc_z_id = 0; acc_z_id <= acc_cnts; ++acc_z_id){
                Vector3d *state_t0 = new Vector3d[3];
                Vector3d *state_tn = new Vector3d[3];
                // todo: tennis ball is not static
                state_t0[0] = Vector3d(m_tennis_racket_1_odom.pose.pose.position.x,
                                       m_tennis_racket_1_odom.pose.pose.position.y,
                                       m_tennis_racket_1_odom.pose.pose.position.z);
                // todo: set state_t0 velocity and acceleration according to robot sensor
                state_t0[1] = Vector3d(0.0, 0.0, 0.0);
                state_t0[2] = Vector3d(0.0, 0.0, 0.0);
                state_tn[0] = Vector3d(m_tennis_ball_odom.pose.pose.position.x,
                                       m_tennis_ball_odom.pose.pose.position.y,
                                       m_tennis_ball_odom.pose.pose.position.z);
                state_tn[1] = Vector3d(vel_x_id, vel_y_id, vel_z_id) * vel_res
                  + Vector3d(vel_min, vel_min, vel_min);
                state_tn[2] = Vector3d(acc_x_id, acc_y_id, acc_z_id) * acc_res
                  + Vector3d(acc_min, acc_min, acc_min);
                MotionPrimitives *traj_primitive_ptr = new MotionPrimitives();
                traj_primitive_ptr->inputTrajectoryParam(5, period_time, state_t0, state_tn);
                double traj_cost = traj_primitive_ptr->getTrajectoryJerkCost();
                if (min_cost < 0 || traj_cost < min_cost){
                  min_cost = traj_cost;
                  (*m_traj_primitive_ptr) = (*traj_primitive_ptr);
                }
              }
    if (min_cost < 0){
      ROS_ERROR("[HybridPlannar] Could not find the mininum cost trajectory.");
      return false;
    }
    else{
      ROS_INFO("[HybridPlannar] Mininum cost trajectory is generated.");
      return true;
    }
  }

  void HybridPlannar::snakeOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    m_snake_odom = *msg;
    tf::StampedTransform transform;
    geometry_msgs::Twist link_twist;
    m_tf_listener.lookupTransform("/world", "/link_racket_1_center", ros::Time(0), transform);
    m_tf_listener.lookupTwist("/world", "/link_racket_1_center", ros::Time(0), ros::Duration(0.1), link_twist);
    m_tennis_racket_1_odom.pose.pose.position.x = transform.getOrigin().x();
    m_tennis_racket_1_odom.pose.pose.position.y = transform.getOrigin().y();
    m_tennis_racket_1_odom.pose.pose.position.z = transform.getOrigin().z();
    tf::Quaternion q;
    transform.getBasis().getRotation(q);
    m_tennis_racket_1_odom.pose.pose.orientation.x = q.getX();
    m_tennis_racket_1_odom.pose.pose.orientation.y = q.getY();
    m_tennis_racket_1_odom.pose.pose.orientation.z = q.getZ();
    m_tennis_racket_1_odom.pose.pose.orientation.w = q.getW();
    m_tennis_racket_1_odom.twist.twist = link_twist;
  }

  void HybridPlannar::tennisBallOdomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    m_tennis_ball_odom = *msg;
    visualizeTennisBall();
    usleep(50000);
  }

  void HybridPlannar::visualizeTennisBall()
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker tennis_ball_marker;
    tennis_ball_marker.ns = "tennis_ball";
    tennis_ball_marker.header.frame_id = std::string("/world");
    tennis_ball_marker.header.stamp = ros::Time().now();
    tennis_ball_marker.action = visualization_msgs::Marker::ADD;
    tennis_ball_marker.type = visualization_msgs::Marker::SPHERE;
    tennis_ball_marker.pose = m_tennis_ball_odom.pose.pose;
    tennis_ball_marker.lifetime = ros::Duration(0.05);

    tennis_ball_marker.scale.x = 0.2;
    tennis_ball_marker.scale.y = 0.2;
    tennis_ball_marker.scale.z = 0.2;
    tennis_ball_marker.color.a = 1.0;
    tennis_ball_marker.color.r = 1.0f;
    tennis_ball_marker.color.g = 1.0f;
    tennis_ball_marker.color.b = 0.0f;
    markers.markers.push_back(tennis_ball_marker);

    m_pub_tennis_ball_markers.publish(markers);
  }

  void HybridPlannar::visualizeTrajectory()
  {
    nav_msgs::Path traj_path;
    traj_path.header.frame_id = std::string("/world");
    traj_path.header.stamp = ros::Time().now();
    float sample_gap = 0.02f;
    traj_path.poses.clear();
    int n_sample = int(m_traj_primitive_ptr->m_traj_period_time / sample_gap);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = traj_path.header;
    pose_stamped.pose.orientation.x = 0.0f;
    pose_stamped.pose.orientation.y = 0.0f;
    pose_stamped.pose.orientation.z = 0.0f;
    pose_stamped.pose.orientation.w = 1.0f;

    //tinyspline::BSpline beziers = m_spline_ptr->derive().toBeziers();

    for (int i = 0; i <= n_sample; ++i){
      Vector3d result = m_traj_primitive_ptr->getTrajectoryPoint(i*sample_gap, 0);
      pose_stamped.pose.position.x = result[0];
      pose_stamped.pose.position.y = result[1];
      pose_stamped.pose.position.z = result[2];
      traj_path.poses.push_back(pose_stamped);
    }
    m_pub_snake_traj_path.publish(traj_path);
  }
}
