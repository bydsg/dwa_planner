#ifndef _MAIN_H
#define _MAIN_H

#include "headfile.h"
#define ODOM

struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;

};
struct State{
        double x = 0;          // m
        double y = 0;          // m
        double yaw = 0;        // degree
        double speed = 0;      // m/s
        double yawrate = 0;
    };
 
class pure_pursuit
{
public:
    ros::Publisher pub_control_cmd;
    ros::Publisher Tracking_point;
    ros::Publisher all_obs;
    ros::Publisher dwa_obs;
    ros::Publisher dwa_point;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher selected_trajectory_pub;

    std::vector<Point> controlRoadPoints;
    State vehicleState;
    std::vector<Box_2d> obstacles_;
    std::vector<int> obstacle_to_road_nearest_index;
    std::vector<Box_2d> obstacles_choose;
    std::vector<int> speed_obstacle_to_road_nearest_index;
    int goalIndex = 0;
    double speed=0;

    void nodeStart(int argc, char **argv);
    void globlepathGetCallBack(by_djstl::Path msg);
  
   // void  control_odometryGetCallBack_GPS(const  geometry_msgs::PoseStamped::ConstPtr odometry_msg);
    int  get_nearest_index(const State &s, std::vector<Point> &path);
    void track_follow_process(const State &s,std::vector<Point> &pathVet);
    double pure_pursuit_control(const State &s, std::vector<Point> &path);
    int get_goal_index(const State &s, std::vector<Point> &path);
    void DWA_planning_process( std::vector<Point> &pathVet,State vehicleState,Box_2d  vel_info, std::vector<int> obstacle_to_road_nearest_index, std::vector<Box_2d> obstacles_choose);
    int get_multi_goal_index(double speed,  int &index, std::vector<Point> &path,double l,double k,int far_obs_index);
    vector<State_DWA> dwa_planning(State vehicleState,  Box_2d  vel_info, Window dynamic_window,  Eigen::Vector3d goal, std::vector<Box_2d> obstacles_choose);
 
    void control_all(State vehicleState);
    void  vehicle_speed_GetCallBack(geometry_msgs::TwistStamped msg);

#ifdef GPS
    void pure_pursuit::vehicle_speed_GetCallBack(geometry_msgs::TwistStamped msg)
    {
        double g_velocity = std::sqrt(std::pow(msg.twist.linear.x, 2) + std::pow(msg.twist.linear.y, 2));
        this->vehicleState.speed = g_velocity;
        this->vehicleState.yawrate = msg.twist.angular.z;
    }
    void control_odometryGetCallBack_GPS(const  geometry_msgs::PoseStamped::ConstPtr odometry_msg);
    void get_odom_msge(const  geometry_msgs::PoseStamped::ConstPtr odometry_msg)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        this->vehicleState.x = odometry_msg->pose.position.x;
        this->vehicleState.y = odometry_msg->pose.position.y;
        this->vehicleState.yaw = theta;
    }
#else ifdef ODOM
    void control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void get_odom_msge(const nav_msgs::Odometry::ConstPtr odometry_msg)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        double g_velocity = std::sqrt(std::pow(odometry_msg->twist.twist.linear.x, 2) + std::pow(odometry_msg->twist.twist.linear.y, 2));
        this->vehicleState.x = odometry_msg->pose.pose.position.x;
        this->vehicleState.y = odometry_msg->pose.pose.position.y;
        this->vehicleState.speed = g_velocity;
        this->vehicleState.yaw = theta;
        this->vehicleState.yawrate = odometry_msg->twist.twist.angular.z;
    }
#endif
};

  

 double calcDistance(const State &start, std::vector<Point> pathVet)
	{
        int num = pathVet.size();
		double x = pathVet[num-1].x- start.x;
        double y = pathVet[num-1].y- start.y;
		return sqrt(x*x+ y*y);
	};

    void range_precote(auto &val,double max,double min){
        if(val>max) {val=max;}
        if(val<min) {val=min;}
    }

#endif