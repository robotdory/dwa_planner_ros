
#ifndef DWA_PLANNER_DWA_PLANNER_H
#define DWA_PLANNER_DWA_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>


class DWAPlanner
{
public:

     DWAPlanner(void);

    class State
    {
    public:
   
        State(void);
  
        State(const double x, const double y, const double yaw, const double velocity, const double yawrate);

        double x_;
        double y_;
        double yaw_;
        double velocity_;
        double yawrate_;
        double direction_to_goal_;
        
    private:
    };

 
    class Window
    {
    public:
  
        Window(void);

        Window(
            const double min_velocity, const double max_velocity, const double min_yawrate, const double max_yawrate);

        double min_velocity_;
        double max_velocity_;
        double min_yawrate_;
        double max_yawrate_;

    private:
    };


    class Cost
    {
    public:
 
        Cost(void);

        Cost(const float obs_cost, const float dist_cost, const float direction_cost, const float total_cost);
    
        void show(void);

        void calc_total_cost(void);

        float obs_cost_;
        float dist_cost_;
        float velocity_cost_;
        float total_cost_;
        
    private:
    };

    struct GoalPoint {
    double x;
    double y;
    double yaw;
    };
    
    std::vector<GoalPoint> goals = {{goal_x_1, goal_y_1, goal_yaw_1}, 
                                    {goal_x_2, goal_y_2, goal_yaw_2}};



    void process(void);

    void scan_callback(const sensor_msgs::LaserScanConstPtr &msg);

    Window create_dynamic_window(void);

    float calc_obstacle_cost(const std::vector<State> &traj);
    float calc_dist_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
    float calc_velocity_cost(const std::vector<State> &trajectory, const double prev_yawrate);
    float calc_dist_to_path(const State state);

    void state_simulation(State &state, const double velocity, const double yawrate);
    void scan_obstacles(const sensor_msgs::LaserScan &scan);
    void update_local_goal();


    std::vector<State> generate_trajectory(const double velocity, const double yawrate);
    std::vector<State> generate_trajectory(const double yawrate, const Eigen::Vector3d &goal);
    Cost evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal);
    geometry_msgs::Twist move_robot(void);

   
    geometry_msgs::Twist align_robot(void);

    void normalize_cost(std::vector<Cost> &costs);

    std::vector<State>

    dwa_planner(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories);

    bool calculate_pose(tf::StampedTransform &robot_transform);
    
public:
    std::string robot_frame_;
    
    double obs_th_; 
    double max_velocity_;
    double min_velocity_;
    double max_yawrate_;
    double max_acceleration_;
    double max_deceleration_;
    double max_d_yawrate_;
    double predict_time_;
    double dt_;
    double obs_cost_gain_;
    double dist_cost_gain_;
    double velocity_cost_gain_;
    double dist_to_goal_th_;
    double ang_to_goal_th_;
    double obs_range_;
    double initial_dist_cost_gain;
    double modified_r;

    double robot_x;
    double robot_y;
    double robot_yaw;

    double goal_x;
    double goal_y;
    double goal_yaw;

    double goal_x_1;
    double goal_y_1;
    double goal_yaw_1;

    double goal_x_2;
    double goal_y_2;
    double goal_yaw_2;

    double goal_angle_diff;
    double distance_diff;
    double target_angle_diff;
    double target_yaw;
    double prev_yawrate;

    bool scan_updated_;
    bool robot_reached_;
    bool robot_aligned_;
    bool local_goal_arrived_;
    int velocity_samples_;
    int yawrate_samples_;
    int scan_count;
    int goal_index;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;

    ros::Publisher velocity_pub_;
    ros::Subscriber scan_sub_;

    geometry_msgs::Twist current_cmd_vel_;
    geometry_msgs::PoseStamped goal_;
    geometry_msgs::PoseArray obs_list_;

    tf::TransformListener tf_listener;
    tf::StampedTransform robot_transform;
    
};

#endif 
