#ifndef HYBRID_PLANNAR_H_
#define HYBRID_PLANNAR_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/* local library */
#include <aerial_robot_base/FlightNav.h>
#include <tennis_hybrid_plannar/SnakeCommand.h>

using namespace Eigen;
using namespace snake_command;

namespace hybrid_plannar{
  class HybridPlannar
  {
  public:
    HybridPlannar(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~HybridPlannar(){}

  private:
    nav_msgs::Odometry m_snake_odom;
    bool m_task_start_flag;
    /* Subscriber */
    ros::NodeHandle m_nh, m_nhp;
    ros::Subscriber m_sub_snake_task_start_flag;
    ros::Subscriber m_sub_snake_odom;
    ros::Subscriber m_sub_tennis_ball_odom;

    /* Publisher */
    ros::Publisher m_pub_snake_start_flag;
    ros::Publisher m_pub_snake_takeoff_flag;
    ros::Publisher m_pub_snake_land_flag;
    ros::Publisher m_pub_snake_joint_states;
    ros::Publisher m_pub_snake_flight_nav;
    ros::Publisher m_pub_tennis_ball_markers;
    ros::Publisher m_pub_tennis_table_markers;
    ros::Publisher m_pub_snake_traj_path;

    /* Topic name */
    std::string m_sub_snake_odom_topic_name;
    std::string m_sub_tennis_ball_odom_topic_name;
    std::string m_pub_snake_start_flag_topic_name;
    std::string m_pub_snake_takeoff_flag_topic_name;
    std::string m_pub_snake_land_flag_topic_name;
    std::string m_pub_snake_joint_states_topic_name;
    std::string m_pub_snake_flight_nav_topic_name;
    std::string m_pub_snake_traj_path_topic_name;

    /* Snake states */
    int m_snake_traj_order;
    int m_n_snake_links;
    double m_snake_link_length;
    double m_snake_average_vel;
    double *m_snake_joint_states_vel_ptr;
    double *m_snake_joint_states_ang_ptr;
    SnakeCommand *m_snake_command_ptr;
    tf::TransformListener m_tf_listener;

    /* Trajectory */
    MotionPrimitives *m_traj_primitive_ptr;

    /* Tennis */
    nav_msgs::Odometry m_tennis_ball_odom;
    nav_msgs::Odometry m_tennis_racket_1_odom;
    // todo: static cheat mode
    bool m_tennis_ball_static_hit_mode;

    void snakeInitPose();
    void taskStartCallback(std_msgs::Empty msg);
    void snakeOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void tennisBallOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    bool generateTrajectory();
    void visualizeTennisBall();
    void visualizeTennisTable();
    bool getFesibleTrajectory();
    void visualizeTrajectory();
  };
}
#endif
