#ifndef IMPROVED_DWA_LOCAL_PLANNER_H_
#define IMPROVED_DWA_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <angles/angles.h>

namespace improved_dwa_local_planner {

class ImprovedDWALocalPlanner : public nav_core::BaseLocalPlanner {
public:
    ImprovedDWALocalPlanner();
    ~ImprovedDWALocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:
    // Структура для хранения информации о траектории
    struct Trajectory {
        double vx;
        double vth;
        double cost;
        std::vector<geometry_msgs::PoseStamped> poses;
    };

    // Вспомогательные функции
    Trajectory computeTrajectory(double vx, double vth);
    double scoreTrajectory(const Trajectory& traj, const geometry_msgs::PoseStamped& local_goal);

    // Переменные-члены класса
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    base_local_planner::OdometryHelperRos odom_helper_;

    // Параметры из статьи (α, β, γ) и другие
    double alpha_, beta_, gamma_;
    double max_vel_x_, min_vel_x_;
    double max_vel_th_;
    double acc_lim_x_, acc_lim_th_;
    double predict_time_;
    double dt_;
};

} // namespace improved_dwa_local_planner

#endif