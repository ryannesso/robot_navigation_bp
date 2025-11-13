#include "improved_dwa_local_planner/improved_dwa_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h> // Явно добавим для ясности

// Регистрация этого планировщика как плагина BaseLocalPlanner
PLUGINLIB_EXPORT_CLASS(improved_dwa_local_planner::ImprovedDWALocalPlanner, nav_core::BaseLocalPlanner)

namespace improved_dwa_local_planner {

ImprovedDWALocalPlanner::ImprovedDWALocalPlanner() : initialized_(false) {}

ImprovedDWALocalPlanner::~ImprovedDWALocalPlanner() {}

void ImprovedDWALocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);

        tf_ = tf;
        costmap_ros_ = costmap_ros;

        // Загрузка параметров с ROS-сервера
        private_nh.param("alpha", alpha_, 0.05);
        private_nh.param("beta", beta_, 0.2);
        private_nh.param("gamma", gamma_, 0.1);
        
        private_nh.param("max_vel_x", max_vel_x_, 1.0);
        private_nh.param("min_vel_x", min_vel_x_, 0.0);
        private_nh.param("max_vel_th", max_vel_th_, 0.35);

        private_nh.param("acc_lim_x", acc_lim_x_, 0.2);
        private_nh.param("acc_lim_th", acc_lim_th_, 0.87);

        private_nh.param("predict_time", predict_time_, 3.0);
        private_nh.param("dt", dt_, 0.1);

        odom_helper_.setOdomTopic("odom");

        initialized_ = true;
        ROS_INFO("Improved DWA Local Planner initialized with parameters: alpha=%.2f, beta=%.2f, gamma=%.2f", alpha_, beta_, gamma_);
    }
}

bool ImprovedDWALocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized.");
        return false;
    }
    global_plan_ = plan;
    return true;
}

bool ImprovedDWALocalPlanner::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized.");
        return false;
    }
    
    // *** ФИНАЛЬНОЕ ИСПРАВЛЕНИЕ ***
    // 1. Получаем текущую позу робота
    geometry_msgs::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
        ROS_WARN("Could not get robot pose, assuming we are not at the goal");
        return false;
    }

    // 2. Получаем текущую одометрию
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    // 3. Вызываем функцию с ПРАВИЛЬНЫМИ аргументами в ПРАВИЛЬНОМ порядке
    // Аргументы: tf, plan, costmap, global_frame, global_pose, base_odom, ...
    return base_local_planner::isGoalReached(*tf_, global_plan_, *(costmap_ros_->getCostmap()),
        costmap_ros_->getGlobalFrameID(), global_pose, base_odom,
        0.01, 0.01, // rot_stopped_vel, trans_stopped_vel
        0.2, 0.2); // xy_goal_tolerance, yaw_goal_tolerance
}

bool ImprovedDWALocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized.");
        return false;
    }

    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);
    geometry_msgs::Twist current_vel = base_odom.twist.twist;
    double current_vx = current_vel.linear.x;
    double current_vth = current_vel.angular.z;
    
    // Расчет динамического окна скоростей
    double min_v = std::max(min_vel_x_, current_vx - acc_lim_x_ * predict_time_);
    double max_v = std::min(max_vel_x_, current_vx + acc_lim_x_ * predict_time_);
    double min_w = std::max(-max_vel_th_, current_vth - acc_lim_th_ * predict_time_);
    double max_w = std::min(max_vel_th_, current_vth + acc_lim_th_ * predict_time_);

    Trajectory best_traj;
    best_traj.cost = -1.0;

    // Находим локальную целевую точку на глобальном плане
    geometry_msgs::PoseStamped local_goal;
    if(!base_local_planner::getGoalPose(*tf_, global_plan_, costmap_ros_->getGlobalFrameID(), local_goal)) {
        ROS_WARN("Could not find local goal from plan");
        return false;
    }

    // Итерация по всем возможным скоростям в динамическом окне
    for (double v = min_v; v <= max_v; v += 0.05) {
        for (double w = min_w; w <= max_w; w += 0.1) {
            Trajectory traj = computeTrajectory(v, w);
            traj.cost = scoreTrajectory(traj, local_goal);
            
            if (traj.cost > best_traj.cost) {
                best_traj = traj;
            }
        }
    }
    
    if (best_traj.cost < 0) {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return false; // Не найдено ни одной валидной траектории
    }
    
    cmd_vel.linear.x = best_traj.vx;
    cmd_vel.angular.z = best_traj.vth;
    return true;
}

ImprovedDWALocalPlanner::Trajectory ImprovedDWALocalPlanner::computeTrajectory(double vx, double vth) {
    Trajectory traj;
    traj.vx = vx;
    traj.vth = vth;
    
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);

    double x = current_pose.pose.position.x;
    double y = current_pose.pose.position.y;
    double theta = tf2::getYaw(current_pose.pose.orientation);
    
    // Симуляция движения на predict_time_ секунд вперед
    for (double t = 0; t < predict_time_; t += dt_) {
        x += vx * cos(theta) * dt_;
        y += vx * sin(theta) * dt_;
        theta += vth * dt_;

        geometry_msgs::PoseStamped pose;
        pose.header = current_pose.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        pose.pose.orientation = tf2::toMsg(q);
        traj.poses.push_back(pose);
    }
    return traj;
}

double ImprovedDWALocalPlanner::scoreTrajectory(const Trajectory& traj, const geometry_msgs::PoseStamped& local_goal) {
    // 1. Стоимость зазора/расстояния до препятствий (Clearance Cost - компонент β)
    double max_traj_cost = 0.0;
    for (const auto& pose : traj.poses) {
        unsigned int cell_x, cell_y;
        if (!costmap_ros_->getCostmap()->worldToMap(pose.pose.position.x, pose.pose.position.y, cell_x, cell_y)) {
            return -1.0;
        }
        unsigned char cost = costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
        
        if (cost >= costmap_2d::LETHAL_OBSTACLE) {
            return -1.0;
        }
        max_traj_cost = std::max(max_traj_cost, (double)cost);
    }
    // Нормализуем относительно максимальной "нелетальной" стоимости.
    double clearance_cost = 1.0 - (max_traj_cost / (double)costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    clearance_cost = std::max(0.0, clearance_cost);

    // 2. Стоимость направления (Heading Cost - компонент α)
    const auto& final_pose = traj.poses.back();
    double final_yaw = tf2::getYaw(final_pose.pose.orientation);
    double goal_angle = atan2(local_goal.pose.position.y - final_pose.pose.position.y,
                              local_goal.pose.position.x - final_pose.pose.position.x);
    double angle_diff = std::abs(angles::shortest_angular_distance(final_yaw, goal_angle));
    double heading_cost = (M_PI - angle_diff) / M_PI;

    // 3. Стоимость скорости (Velocity Cost - компонент γ)
    double velocity_cost = (max_vel_x_ > 0) ? (traj.vx / max_vel_x_) : 0.0;
    
    // Итоговая оценка: взвешенная сумма трех компонентов.
    return alpha_ * heading_cost + beta_ * clearance_cost + gamma_ * velocity_cost;
}

} // namespace