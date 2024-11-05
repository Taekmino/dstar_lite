#include "dummy_node.h"

Dummy::Dummy() {
    _nh.getParam("/subgoal_topic", _subgoal_topic);
    _nh.getParam("/localization_topic", _localization_topic);
    _nh.getParam("/cost_map_topic", _map_topic);
    _nh.getParam("/local_path_topic", _local_path_topic);
    _nh.getParam("/odom_frame_id", _frame_id);
    _nh.getParam("/subgoal_arrived_topic", _subgoal_arrived_topic);
    _nh.getParam("/clicked_goal_topic", _clicked_goal_topic);
    
    _nh.getParam("/dummy/max_size", _max_size);
    _nh.getParam("/dummy/n_obstacles", _n_obstacles);
    _nh.getParam("/dummy/seed", _seed);
    _nh.getParam("/dummy/map_width", _width);
    _nh.getParam("/dummy/map_height", _height);
    _nh.getParam("/dummy/resolution", _resolution);
    _nh.getParam("/dummy/origin_x", _origin_x);
    _nh.getParam("/dummy/origin_y", _origin_y);

    // _go1_loc_sub = _nh.subscribe(_localization_topic, 10, &Dummy::_localizationCallback, this);

    _subgoal_arrived_sub = _nh.subscribe(_subgoal_arrived_topic, 10, &Dummy::_subgoalArrivedCallback, this);
    _path_sub = _nh.subscribe(_local_path_topic, 10, &Dummy::_localPathCallback, this);
    _clicked_goal_sub = _nh.subscribe(_clicked_goal_topic, 10, &Dummy::_clickedGoalCallback, this);
    _dummy_loc_pub = _nh.advertise<nav_msgs::Odometry>(_localization_topic, 100);
    _dummy_map_pub = _nh.advertise<nav_msgs::OccupancyGrid>(_map_topic, 100);
    _dummy_goal_pub = _nh.advertise<geometry_msgs::Point>(_subgoal_topic, 100);
    _goal_vis_pub = _nh.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/subgoal_vis", 100);
    _loc_vis_pub = _nh.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/loc_vis", 100);

    _subgoal_arrived = false;
    diff = false;

    _rng = std::mt19937_64(_seed);       
    std::uniform_real_distribution<double> unif_x(_origin_x, _origin_x + _width * _resolution);
    std::uniform_real_distribution<double> unif_y(_origin_y, _origin_y + _height * _resolution);

    double start_x = unif_x(_rng);
    double start_y = unif_y(_rng);
    double goal_x = unif_x(_rng);
    double goal_y = unif_y(_rng);

    int counter = 0;
    while (sqrt(pow(start_x - goal_x, 2.0) + pow(start_y - goal_y, 2.0)) < _width * _resolution / 2 && counter < 1000) {
        std::cout << "Setting goal..." << std::endl;
        goal_x = unif_x(_rng);
        goal_y = unif_y(_rng);
        counter++;
    }

    _init_pose = std::vector<double>{start_x, start_y, 0.0};
    _goal_pose = std::vector<double>{goal_x, goal_y, 0.0};

    _dummy_loc_msg.pose.pose.position.x = _init_pose[0];
    _dummy_loc_msg.pose.pose.position.y = _init_pose[1];
    _dummy_loc_msg.pose.pose.position.z = _init_pose[2];

    _dummy_subgoal.x = _goal_pose[0];
    _dummy_subgoal.y = _goal_pose[1];
    _dummy_subgoal.z = _goal_pose[2];
    _fillMaps();
}

std::vector<std::vector<int>> Dummy::getRandomDummyObstacle(int n_obstacle, int max_size, int width, int height) {
    std::uniform_int_distribution<int> unif_x(0, width);
    std::uniform_int_distribution<int> unif_y(0, height);
    std::uniform_int_distribution<int> unif_width(max_size/2, max_size);
    std::uniform_int_distribution<int> unif_height(max_size/2, max_size);
    std::vector<std::vector<int>> obstacles;
    for (int i=0; i<n_obstacle; i++) {
        int x_start = unif_x(_rng);
        int y_start = unif_y(_rng);
        int obs_width = unif_width(_rng);
        int obs_height = unif_height(_rng);
        std::vector<int> obstacle{x_start, x_start+obs_width, y_start, y_start+obs_height};

        std::vector<int> grid_loc = _pointToGrid(_dummy_loc_msg.pose.pose.position);
        std::vector<int> grid_goal = _pointToGrid(_dummy_subgoal);
        if (sqrt(pow(x_start + obs_width/2 - grid_loc[0], 2.0) + pow(y_start + obs_width/2 - grid_loc[1], 2.0)) < obs_width ||
        sqrt(pow(x_start + obs_height/2 - grid_goal[0], 2.0) + pow(y_start + obs_height/2 - grid_goal[1], 2.0)) < obs_height) continue;

        obstacles.push_back(obstacle);
    }
    return obstacles;
}

void Dummy::_fillMaps() {
    ros::Time begin = ros::Time::now();

    _dummy_map.info.width = _width;
    _dummy_map.info.height = _height;
    _dummy_map.info.resolution = _resolution;
    _dummy_map.info.origin.position.x = _origin_x;
    _dummy_map.info.origin.position.y = _origin_y;
    _dummy_map.header.stamp = begin;
    _dummy_map.header.frame_id = _frame_id;

    _dummy_local_map.info.width = _width;
    _dummy_local_map.info.height = _height;
    _dummy_local_map.info.resolution = _resolution;
    _dummy_local_map.info.origin.position.x = _origin_x;
    _dummy_local_map.info.origin.position.y = _origin_y;
    _dummy_local_map.header.stamp = begin;
    _dummy_local_map.header.frame_id = _frame_id;

    std::vector<std::vector<int>> obstacles = getRandomDummyObstacle(_n_obstacles, _max_size, _dummy_map.info.width, _dummy_map.info.height);
    _dummy_map.data.clear();
    for (int y=0; y<_dummy_map.info.height; y++) {
        for(int x=0; x<_dummy_map.info.width; x++) {
            _dummy_map.data.push_back(0);
            _dummy_local_map.data.push_back(0);
        }
    }
    for (int obs_idx=0; obs_idx<obstacles.size(); obs_idx++) {
        std::vector<int> obstacle = obstacles[obs_idx];
        for (int x = obstacle[0]; x <= std::min(obstacle[1], static_cast<int>(_dummy_map.info.width)); x++) {
            for (int y = obstacle[2]; y <= std::min(obstacle[3], static_cast<int>(_dummy_map.info.height)); y++) {
                _dummy_map.data[x + _dummy_map.info.width * y] = 100;
            }
        }
    }
}

void Dummy::_localPathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    _dummy_loc_msg.header.frame_id = _frame_id;
    if (path_msg->poses.size() > 1) _dummy_loc_msg.pose.pose = path_msg->poses[1].pose;
    if (_subgoal_arrived == true) {
        ROS_INFO("[DUMMY LOCALIZATION] GOAL ARRIVED");
        // _arriveGoal();
    }
}

void Dummy::_clickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg) {
    _dummy_subgoal.x = subgoal_msg->point.x;
    _dummy_subgoal.y = subgoal_msg->point.y;
    _dummy_subgoal.z = subgoal_msg->point.z;
}

void Dummy::_subgoalArrivedCallback(const std_msgs::Bool::ConstPtr& subgoal_arrived_msg) {
    if (subgoal_arrived_msg->data == true) {
        _subgoal_arrived = true;
    } else {
        _subgoal_arrived = false;
    }
}

void Dummy::_arriveGoal(){
    _dummy_loc_msg.pose.pose.position.x = _goal_pose[0];
    _dummy_loc_msg.pose.pose.position.y = _goal_pose[1];
    _dummy_loc_msg.pose.pose.position.z = _goal_pose[2];

    std::uniform_real_distribution<double> unif_x(_origin_x, _origin_x + _width * _resolution);
    std::uniform_real_distribution<double> unif_y(_origin_y, _origin_y + _height * _resolution);

    double goal_x = unif_x(_rng);
    double goal_y = unif_y(_rng);
    while (true) {
        std::cout << "Setting goal..." << std::endl;
        goal_x = unif_x(_rng);
        goal_y = unif_y(_rng);
        geometry_msgs::Point point;
        point.x = goal_x;
        point.y = goal_y;
        std::vector<int> grid_xy = _pointToGrid(point);

        if (static_cast<int>(_dummy_map.data[grid_xy[0] + _dummy_map.info.width * grid_xy[1]]) == 0 
        && static_cast<int>(_dummy_map.data[grid_xy[0] + 1+ _dummy_map.info.width * grid_xy[1]]) == 0
        && static_cast<int>(_dummy_map.data[grid_xy[0] - 1+ _dummy_map.info.width * grid_xy[1]]) == 0
        && sqrt(pow(_goal_pose[0] - goal_x, 2.0)
         + pow(_goal_pose[1] - goal_y, 2.0)) > _width * _resolution / 2) break;
    }

    _goal_pose = std::vector<double>{goal_x, goal_y, 0.0};
    _subgoal_arrived = false;
    
    _dummy_subgoal.x = _goal_pose[0];
    _dummy_subgoal.y = _goal_pose[1];
    _dummy_subgoal.z = _goal_pose[2];
}

void Dummy::publishTopics() {
    _dummy_loc_pub.publish(_dummy_loc_msg);
    _dummy_map_pub.publish(_dummy_map);
    _dummy_goal_pub.publish(_dummy_subgoal);

    _visualizeLocGoals();
}

std::vector<int> Dummy::_pointToGrid(geometry_msgs::Point point) {
    int grid_x = (point.x - _dummy_map.info.origin.position.x) / _dummy_map.info.resolution;
    int grid_y = (point.y - _dummy_map.info.origin.position.y) / _dummy_map.info.resolution;
    std::vector<int> grid_xy{grid_x, grid_y};
    return grid_xy;
}

void Dummy::_visualizeLocGoals() {
    visualization_msgs::Marker goal_vis, loc_vis;
    goal_vis.type = goal_vis.CYLINDER;
    goal_vis.scale.x = 0.2;
    goal_vis.scale.y = 0.5;
    goal_vis.scale.z = 0.2;
    goal_vis.color.r = 1.0;
    goal_vis.color.g = 0.0;
    goal_vis.color.b = 0.0;
    goal_vis.color.a = 1.0;
    goal_vis.header.frame_id = _frame_id;
    goal_vis.pose.position = _dummy_subgoal;
    goal_vis.pose.orientation.x = 0;
    goal_vis.pose.orientation.y = 0;
    goal_vis.pose.orientation.z = 0;
    goal_vis.pose.orientation.w = 1;

    loc_vis.type = loc_vis.CYLINDER;
    loc_vis.scale.x = 0.2;
    loc_vis.scale.y = 0.5;
    loc_vis.scale.z = 0.2;
    loc_vis.color.r = 0.0;
    loc_vis.color.g = 1.0;
    loc_vis.color.b = 0.0;
    loc_vis.color.a = 1.0;
    loc_vis.header.frame_id = _frame_id;
    loc_vis.pose.position = _dummy_loc_msg.pose.pose.position;
    loc_vis.pose.orientation.y = 0;
    loc_vis.pose.orientation.z = 0;
    loc_vis.pose.orientation.w = 0;
    loc_vis.pose.orientation.x = 1;

    _goal_vis_pub.publish(goal_vis);
    _loc_vis_pub.publish(loc_vis);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "DummyNodes");
    Dummy dummy;
    ROS_INFO("DummyNodes initialized.");
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();

        dummy.publishTopics();
        loop_rate.sleep();
    }
}

