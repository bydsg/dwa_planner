#include "include/headfile.h"

vector<State_DWA> pure_pursuit::dwa_planning(State vehicleState,  Box_2d  vel_info, Window dynamic_window,  Eigen::Vector3d goal, std::vector<Box_2d> obstacles_choose)
{   
    float min_cost= 1e6;
    float min_obs_cost = 1e6;
    float min_goal_cost = 1e6;
    float min_speed_cost = 1e6;

    std::vector<std::vector<State_DWA>> trajectories;
    std::vector<State_DWA> best_traj;

    for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
        for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
            State_DWA state(vehicleState.x, vehicleState.y, vehicleState.yaw, vehicleState.speed, vehicleState.yawrate);////
            std::vector<State_DWA> traj;
            for(float t=0; t<=PREDICT_TIME; t+=DT){
                motion(state, v, y);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            float to_goal_cost = calc_to_goal_cost(traj, goal);
            float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
            float obstacle_cost = calc_obstacle_cost(traj, obstacles_choose);
            float final_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;
            if(min_cost >= final_cost){
                min_goal_cost = TO_GOAL_COST_GAIN*to_goal_cost;
                min_obs_cost = OBSTACLE_COST_GAIN*obstacle_cost;
                min_speed_cost = SPEED_COST_GAIN*speed_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }
    visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
    if(min_cost == 1e6){
        std::vector<State_DWA> traj;
        State_DWA state(vehicleState.x, vehicleState.y, vehicleState.yaw, vehicleState.speed, vehicleState.yawrate);////
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}
int pure_pursuit::get_multi_goal_index(double speed,  int &index, std::vector<Point> &path,double l,double k,int far_obs_index){
        double l0= 0;
        double lf = k * speed + obstacles_choose[far_obs_index].half_diagonal()+l;
        double dx_, dy_; 
        //积分法计算路径长度
        while (l0 < lf && index < path.size()) {
            dx_ = path[index + 1].x - path[index].x;
            dy_ = path[index + 1].y - path[index].y;
            l0 += sqrt(dx_ * dx_ + dy_ * dy_);
            index++;
        }
        return index;
    } 
void pure_pursuit::DWA_planning_process( std::vector<Point> &pathVet,State vehicleState,Box_2d  vel_info, std::vector<int> obstacle_to_road_nearest_index, std::vector<Box_2d> obstacles_choose){
    int max_road_index = 0; // 路经点下标
    int far_obs_index;      // 障碍物下标
    // 找到距离小车最远的路径点下标和那个障碍物
    for (int i = 0; i < obstacle_to_road_nearest_index.size(); i++)
    {
        if (obstacle_to_road_nearest_index[i] > max_road_index)
        {
            max_road_index = obstacle_to_road_nearest_index[i];
            far_obs_index = i;
        }
    }
    int road_goal_index = get_multi_goal_index(vehicleState.speed, max_road_index, this->controlRoadPoints, l_obs, k_obs,far_obs_index); 
    dwa_point_rviz(dwa_point,controlRoadPoints[road_goal_index]);
    
    Window dynamic_window = calc_dynamic_window(vehicleState.speed, vehicleState.yawrate);
    Eigen::Vector3d goal(this->controlRoadPoints[road_goal_index].x, this->controlRoadPoints[road_goal_index].y, this->controlRoadPoints[road_goal_index].theta);
    geometry_msgs::Twist cmd_vel;
    std::vector<State_DWA> best_traj = dwa_planning(vehicleState, vel_info, dynamic_window, goal, this->obstacles_choose);
    cmd_vel.linear.x = best_traj[0].velocity;
    cmd_vel.angular.z = best_traj[0].yawrate;

    // 判断路径是否走完,如果走完，则急停
    double distance = calcDistance(this->vehicleState, pathVet);
    // double diatance2 = calcDistance2(this->vehicleState, pathVet);
    if (road_goal_index >= (pathVet.size() - 1) && distance <= 0.3)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    visualize_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);
    this->pub_control_cmd.publish(cmd_vel);
}
int pure_pursuit::get_goal_index(const State &s, std::vector<Point> &path) {
        vector<double> d;
        for (int i = 0; i < path.size(); i++)
        {
            d.push_back(pow((s.x - path[i].x), 2) + pow((s.y - path[i].y), 2)); // 距离计算
        }
        int index = 0;
        double d_min = d[0];
        //找到距离车辆最近的路径点
        for (int i = 0; i < d.size(); i++) {
            if (d_min > d[i]) {
                d_min = d[i];
                index = i;//记录下距离车最近的点的index和最近距离
            }
        }
        double l = 0;
        double lf = K0 * s.speed + L0;//前视距离
        double dx_, dy_;
 
        //积分法计算路径长度
        while (l < lf && index < path.size()-1 ) {
            dx_ = path[index + 1].x - path[index].x;
            dy_ = path[index + 1].y - path[index].y;
            l += sqrt(dx_ * dx_ + dy_ * dy_);
            index++;
        }
 
        return index;//这个index代表的就是距离车最近点的距离加上前视距离的坐标点的下标
    }
double pure_pursuit::pure_pursuit_control(const State &s, std::vector<Point> &path) {
        int index =pure_pursuit::get_goal_index(s, path); // 在车当前最近点加上前视距离的道路的坐标点
        // 用上一个循环的目标点判断是否是在向前走
       if (index < this-> goalIndex ) {
            index = this-> goalIndex;
        } 
        Point goal; 
        range_precote(index, path.size() - 1, 0);   //防止index溢出
        goal = path[index];
        rviz_Track_point(Tracking_point,goal);
        double alpha = atan2(goal.y - s.y, goal.x - s.x) - s.yaw; // 误差角度
        if(alpha<-3.14) {alpha=alpha+6.28;}
        if(alpha>3.14) {alpha=alpha-6.28;}

        double lf = K0 * s.speed + L0; // 根据车速和道路曲率设置前视距离
        // delta 即为纯跟踪算法的最终输出
        double delta = atan2((2.0 * L_VEHICLE * sin(alpha)) / lf, 1.0);
        this-> goalIndex = index;      // 为下一个循环更新上一个目标点
        return alpha;
    }
void pure_pursuit::track_follow_process(const State &s,std::vector<Point> &pathVet) {
        int num = pathVet.size();
        int rearIndex = num;
        double wheelAngle = 0;
      
        //判断路径是否走完
       double distance = calcDistance(this->vehicleState, pathVet);
        if ((this->goalIndex >= rearIndex-1)&&( distance<= 0.3)){
            wheelAngle  = 0;
            speed = 0;
         }else{
        if (this->goalIndex < rearIndex) {
        //获取纯跟踪输出的前轮转角
            wheelAngle =pure_pursuit_control(this->vehicleState, pathVet); 
            speed = TARGET_SPEED/abs(wheelAngle);//调整车辆速度
        }
      }      
       geometry_msgs::Twist cmd_vel;
        //发布前轮转角
        cmd_vel.linear.x = speed;
        cmd_vel.linear.y = 0.0;

        cmd_vel.angular.z = wheelAngle;
        range_precote(cmd_vel.angular.z,1,-1);//限幅
        this->pub_control_cmd.publish(cmd_vel);
    }
int pure_pursuit::get_nearest_index(const State &s, std::vector<Point> &path) {
       if(path.size()==0){
        return 0;
       }
       else{
          vector<double> d;
        for (int i = 0; i < path.size(); i++)
            d.push_back(pow((s.x - path[i].x), 2) + pow((s.y - path[i].y), 2));//距离计算
 
        int index = 0;
        double d_min = d[0];
        int dVecLen = d.size();
 
        //找到距离车辆最近的路径点
        for (int i = 0; i < dVecLen; i++) {
            if (d_min > d[i]) {
                d_min = d[i];
                index = i;
            }
        }
        return index;
       }  
    }
void pure_pursuit::globlepathGetCallBack( by_djstl::Path msg){
   this->controlRoadPoints.clear();
     for(int i=0;i<msg.points.size();i++){
         Point p;
         p.x = msg.points[i].x;
         p.y = msg.points[i].y;
         p.l = msg.points[i].l;
         p.r = msg.points[i].r;
        this->controlRoadPoints.push_back(p);
     }
 }
void pure_pursuit::control_all(State vehicleState){
        if( this->controlRoadPoints.size()>0){
        ros::Rate loop_rate(50);
        obstacles_choose.clear();
        obstacles_.clear();
        obstacle_to_road_nearest_index.clear();

        Box_2d vel_info({this->vehicleState.x, this->vehicleState.y}, this->vehicleState.yaw, 0.2, 0.2, 0);
        Box_2d obs1({2.0, 0.0}, 0.0, 0.8, 0.5, 0);
        obstacles_.emplace_back(obs1); 
        all_obs_rviz(all_obs,obstacles_);
     
        for (int i = 0; i < obstacles_.size(); i++)
        {
            State obs;
            obs.x = obstacles_[i].center_x();
            obs.y = obstacles_[i].center_y();
            int index = pure_pursuit::get_nearest_index(obs, this->controlRoadPoints);// 障碍物距离道路最近点的下标
            int car_index = pure_pursuit::get_nearest_index(this->vehicleState, this->controlRoadPoints); // 车距离道路最近点的下标
            double distance_obs_to_road = sqrt(pow((obs.x - this->controlRoadPoints[index].x), 2) + pow((obs.y - this->controlRoadPoints[index].y), 2));
            double distance_obs_to_car = obstacles_[i].DistanceTo(vel_info);
            if (index >= car_index - 2)//如果障碍物在车前（-2给容错）
            {
                //安全距离为：安全距离加障碍物的斜边的一半加车的斜边的一半
                bool lon_unsafe = distance_obs_to_car < lon_safe_distance + vel_info.half_diagonal() + obstacles_[i].half_diagonal() ? true : false;
                // 对车辆产生影响的障碍物
                if (lon_unsafe)
                {
                    this->obstacle_to_road_nearest_index.push_back(index);
                    this->obstacles_choose.push_back(obstacles_[i]);
                }
            }
        }
        dwa_obs_rviz(dwa_obs,obstacles_choose);
         //没有障碍物，寻迹
        if (this->obstacles_choose.size() == 0)
        {
             this->track_follow_process(this->vehicleState, this->controlRoadPoints);
             
        }
        else{
             // 局部避障控制
             this->DWA_planning_process(this->controlRoadPoints, this->vehicleState, vel_info, this->obstacle_to_road_nearest_index, this->obstacles_choose);
        }    
    }
}



    void pure_pursuit::control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
        get_odom_msge(odometry_msg);//获取车的里程计数据
        control_all(vehicleState);
    }




void pure_pursuit::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "planning_control");
    ros::NodeHandle nc;

    ros::Subscriber  sub_globle_path = nc.subscribe("globle_path", 1, &pure_pursuit::globlepathGetCallBack, this);//接受local规划路径
    ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &pure_pursuit::control_odometryGetCallBack, this);//控制节点

    this->pub_control_cmd = nc.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel
    this->Tracking_point = nc.advertise<visualization_msgs::Marker>("Tracking_point", 1);//追踪所标记的点
    this->all_obs = nc.advertise<visualization_msgs::Marker>("all_obs", 1);              // 所有障碍物
    this->dwa_obs = nc.advertise<visualization_msgs::Marker>("dwa_obs", 1);              // dwa考虑的障碍物
    this->dwa_point = nc.advertise<visualization_msgs::Marker>("dwa_point", 1);          // dwa追踪的点
    this->candidate_trajectories_pub = nc.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    this->selected_trajectory_pub = nc.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
    ros::spin();
}


int main(int argc, char  *argv[])
{
    pure_pursuit node;
    node.nodeStart(argc, argv);
    return 0;
}
