#ifndef _RVIZ_H_
#define _RVIZ_H_
#include "headfile.h"
string my_frame_id ="odom";
 void dwa_obs_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info){
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = my_frame_id;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.3;
        // Line list is red
        line_list.color.r = 0.5;
        line_list.color.g = 0.5;
        line_list.color.a = 0.8;
        // Create the vertices for the points
        geometry_msgs::Point p;
        for (uint32_t i = 0; i < vel_obs_info.size(); i++)
        {
            for (uint32_t j = 0; j < vel_obs_info[i].Box2d_corner.size(); j++)
            {
                double y = vel_obs_info[i].Box2d_corner[j].y;
                double x = vel_obs_info[i].Box2d_corner[j].x;
                float z = 0;
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                line_list.points.push_back(p);
            }
        }
        marker_pub.publish(line_list);
    }
    void dwa_point_rviz(ros::Publisher marker_pub, Point point)
    {
        visualization_msgs::Marker points;
        points.header.frame_id = my_frame_id;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.3;
        points.scale.y = 0.3;
        // Points are green
        points.color.a = 1.0; // Don't forget to set the alpha!
        points.color.r = 0.5;
        points.color.g = 0.5;
        points.color.b = 0.5;
        // Create the vertices for the points
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        points.points.push_back(p);
        marker_pub.publish(points);
    }
   void rviz_Track_point(ros::Publisher marker_pub, Point point){ 
    visualization_msgs::Marker points;
    points.header.frame_id = my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    // Points are green
    points.color.a = 1.0; // Don't forget to set the alpha!
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 1.0;
    // Create the vertices for the points 
             geometry_msgs::Point p;
             p.x =  point.x ;
             p.y =  point.y;
             p.z = 0;
             points.points.push_back(p);      
    marker_pub.publish(points);
}
    void all_obs_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info){
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = my_frame_id;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.2;
        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        // Create the vertices for the points
        geometry_msgs::Point p;
        for (uint32_t i = 0; i < vel_obs_info.size(); i++)
        {
            for (uint32_t j = 0; j < vel_obs_info[i].Box2d_corner.size(); j++)
            {
                double y = vel_obs_info[i].Box2d_corner[j].y;
                double x = vel_obs_info[i].Box2d_corner[j].x;
                float z = 0;
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                line_list.points.push_back(p);
            }
        }
        marker_pub.publish(line_list);
}
void visualize_trajectory(const std::vector<State_DWA>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id =  "/camera_init";
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = 0.4;
    v_trajectory.color.g = 0.4;
    v_trajectory.color.b = 0.8;
    v_trajectory.color.a = 1;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    v_trajectory.pose = pose;
    geometry_msgs::Point p;
    for(const auto& pose : trajectory){
        p.x = pose.x;
        p.y = pose.y;
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

#endif