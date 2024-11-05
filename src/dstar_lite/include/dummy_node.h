#pragma once
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include <random>
#include <utility>
#include <algorithm>

class Dummy {
    public:
        Dummy();
        std::vector<std::vector<int>> getRandomDummyObstacle(int n_obstacle, int max_size, int width, int height);
        void publishTopics();

    private:
        void _localPathCallback(const nav_msgs::Path::ConstPtr& path_msg);
        void _subgoalArrivedCallback(const std_msgs::Bool::ConstPtr& subgoal_arrived_msg);
        void _clickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg);
        void _fillMaps();
        // void _FillLocalMaps();
        std::vector<int> _pointToGrid(geometry_msgs::Point point);
        void _visualizeLocGoals();
        void _arriveGoal();

        std::vector<double> _init_pose;
        std::vector<double> _goal_pose;
        
        int _n_obstacles;
        int _max_size;
        int _seed;
        int _width;
        int _height;
        double _resolution;
        double _origin_x;
        double _origin_y;
        bool _subgoal_arrived;
        bool diff;

        std::mt19937_64 _rng;
        geometry_msgs::Point _dummy_subgoal;
        nav_msgs::Odometry _dummy_loc_msg;
        nav_msgs::OccupancyGrid _dummy_map;
        nav_msgs::OccupancyGrid _dummy_local_map;

        ros::NodeHandle _nh;
        
        ros::Subscriber _path_sub;
        ros::Subscriber _subgoal_arrived_sub;
        ros::Subscriber _clicked_goal_sub;
        ros::Publisher _dummy_loc_pub;
        ros::Publisher _dummy_map_pub;
        ros::Publisher _dummy_local_map_pub;
        ros::Publisher _dummy_goal_pub;
        ros::Publisher _goal_vis_pub;
        ros::Publisher _loc_vis_pub;
        
        std::string _subgoal_topic;
        std::string _localization_topic;
        std::string _clicked_goal_topic;
        std::string _map_topic;
        std::string _local_path_topic;
        std::string _frame_id;
        std::string _subgoal_arrived_topic;
};