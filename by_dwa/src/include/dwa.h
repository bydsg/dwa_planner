#ifndef _DWA_H_
#define _DWA_H_

#include "headfile.h"

using namespace std;
#define TARGET_SPEED 0.03//目标车速
#define  K0  0.5     // 前视距离系数
#define L_VEHICLE   2 //车长度
#define  L0  1.0//zui_duan_qian_shi_ju_li
#define k_obs 2
#define l_obs 1
#define MIN_VELOCITY 0.0
#define MAX_VELOCITY 0.6
#define MAX_YAWRATE 0.8
#define MAX_ACCELERATION 1.0
#define DT 1
#define MAX_D_YAWRATE 2.0
#define VELOCITY_RESOLUTION 0.1//步长
#define YAWRATE_RESOLUTION 0.1
#define PREDICT_TIME 3.0
#define TARGET_VELOCITY  0.8 //msg->linear.x
#define TO_GOAL_COST_GAIN 1
#define SPEED_COST_GAIN 1
#define OBSTACLE_COST_GAIN 1
#define GOAL_THRESHOLD 0.3
#define TURN_DIRECTION_THRESHOLD 1
#define SAFE_DISTANCE 0.2
#define half_road_safe_distance  0.2
#define lon_safe_distance  2

    class Window
    {
    public:
        Window(void);
        Window(const double, const double, const double, const double);
        double min_velocity;
        double max_velocity;
        double min_yawrate;
        double max_yawrate;
    private:
    };

      class State_DWA
    {
    public:
        State_DWA(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double velocity;// robot linear velocity
        double yawrate;// robot angular velocity
    private:
    };

State_DWA::State_DWA(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    :x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

Window::Window(void)
    :min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    :min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}


Window calc_dynamic_window(const double& current_velocity, const double& current_yawrate)
{
    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
    window.min_velocity = std::max((current_velocity- MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_velocity = std::min((current_velocity + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_yawrate = std::max((current_yawrate - MAX_D_YAWRATE*DT), -MAX_YAWRATE);
    window.max_yawrate = std::min((current_yawrate+ MAX_D_YAWRATE*DT),  MAX_YAWRATE);
    return window;
}

void motion(State_DWA& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate*DT;
    state.x += velocity*std::cos(state.yaw)*DT;
    state.y += velocity*std::sin(state.yaw)*DT;
    state.velocity = velocity;
    state.yawrate = yawrate;
}
float calc_to_goal_cost(const std::vector<State_DWA>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float calc_speed_cost(const std::vector<State_DWA>& traj, const float target_velocity)
{
    float cost = fabs(target_velocity - fabs(traj[traj.size()-1].velocity));
    return cost;
}

float calc_obstacle_cost(const std::vector<State_DWA>& traj, const std::vector<Box_2d> choose)
{

    float cost = 0.0;
    float min_dist = 1e3;
    for(const auto& state: traj){
        Box_2d  tri_state({state.x , state.y}, state.yaw, 0.4, 0.3, 0.0);
        for(const auto& obs : choose){
            float dist = tri_state.DistanceTo(obs);
            if(dist <= (tri_state.half_diagonal()+obs.half_diagonal()+SAFE_DISTANCE)){
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
        }
    }
    cost = 1.0 / min_dist;
    return cost;
}
void visualize_trajectories(const std::vector<std::vector<State_DWA>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = "/odom";
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        v_trajectory.pose = pose;
        geometry_msgs::Point p;
        for(const auto& pose : trajectories[count]){
            p.x = pose.x;
            p.y = pose.y;
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id =  "/map";
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}




#endif