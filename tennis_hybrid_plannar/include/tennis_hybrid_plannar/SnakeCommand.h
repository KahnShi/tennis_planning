#ifndef SNAKE_COMMAND_H_
#define SNAKE_COMMAND_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>

/* math */
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* local library */
#include <aerial_robot_base/FlightNav.h>
#include <aerial_robot_base/DesireCoord.h>
#include <tennis_hybrid_plannar/MotionPrimitives.h>

#include <iostream>
using namespace Eigen;
using namespace motion_primitives;

namespace snake_command{
  class SnakeCommand
  {
  public:
    SnakeCommand(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~SnakeCommand(){}
    ros::NodeHandle m_nh, m_nhp;
    bool m_move_start_flag;
    int m_n_links;
    double m_link_length;
    double m_control_period;
    tf::Vector3* m_links_pos_ptr;
    tf::Vector3* m_links_vel_ptr;
    double* m_joints_ang_ptr;
    double* m_joints_ang_vel_ptr;
    nav_msgs::Odometry m_base_link_odom;
    tf::Vector3 m_base_link_pos;
    tf::Vector3 m_base_link_vel;
    tf::Vector3 m_base_link_ang;

    std::string m_pub_flight_nav_topic_name;
    std::string m_pub_joints_ctrl_topic_name;
    std::string m_sub_move_start_flag_topic_name;
    std::string m_sub_joint_states_topic_name;
    std::string m_sub_base_link_odom_topic_name;

    tf::TransformListener m_tf_listener;
    double m_cog_world_rpy[3];
    double m_snake_acceleration_max_value;

    /* Trajectory */
    double m_traj_start_time;
    double m_traj_current_time;
    tf::Vector3 m_traj_track_i_term_accumulation;
    MotionPrimitives *m_traj_primitive;
    double m_traj_fixed_yaw;
    int m_traj_robot_state;
    uint8_t m_traj_track_state;
    static constexpr uint8_t TRAJ_TRACK_NOT_START = 1;
    static constexpr uint8_t TRAJ_TRACK_ON_GOING = 2;
    static constexpr uint8_t TRAJ_TRACK_FINISH = 3;
    double m_traj_finish_height;
    double m_traj_control_z_offset;

    /* tennis */
    tf::Vector3 m_racket_1_pos;
    tf::Vector3 m_racket_1_base_link_offset;
    tf::Vector3 m_racket_2_pos;
    int m_racket_state;
    static constexpr uint8_t RACKET_RELEX = 1;
    static constexpr uint8_t RACKET_COMPRESS = 2;
    static constexpr uint8_t RACKET_HIT = 3;
    // todo: static cheat mode
    bool m_tennis_ball_static_hit_mode;

    /* Publisher */
    ros::Publisher m_pub_flight_nav;
    ros::Publisher m_pub_joints_ctrl;
    ros::Publisher m_pub_racket_center_expected_markers;
    // todo: cheat mode
    ros::Publisher m_pub_ball_start_odom;

    /* Subscriber */
    ros::Subscriber m_sub_move_start_flag;
    ros::Subscriber m_sub_joint_states;
    ros::Subscriber m_sub_base_link_odom;
    ros::Subscriber m_sub_cog_world_coord;
    ros::Timer m_timer;

    void initializeVariable();
    void moveStartFlagCallback(const std_msgs::Empty msg);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg);
    void baseLinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void controlCallback(const ros::TimerEvent& e);
    void cogWorldCoordCallback(const aerial_robot_base::DesireCoordConstPtr& coord_msg);
    void directTrackGlobalTrajectory();
    void transformTrackGlobalTrajectory();
    inline tf::Vector3 vectorToVector3(std::vector<double> vec);
    inline tf::Vector3 vector3dToVector3(Vector3d vec);
    tf::Vector3 attitudeCvtWorldToCog(tf::Vector3 att_world);
    void visualizeRacketExpectedPosition(tf::Vector3 des_pos);
  };
}


#endif
