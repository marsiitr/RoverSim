#ifndef PP_PLUGIN_H_
#define PP_PLUGIN_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <path_planning_msg/pp_msg.h>

namespace global_planner {
    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:
            // Standard functions for the plugin

            // Default constructor
            GlobalPlanner();
            // Parametrized constructor
            GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            // Function to initial all nodes and variables
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            // Gets plan from algorithm and publishes it
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
            
            //Additional functions for the plugin

            /*
                Checks if the given position is valid or not
                Parameters: x coordinate and y coordinate of the point that is to be checked
                Return: Returns a boolean (t/f)
            */
            bool isPositionValid(double, double);

            /*
                Converts the world coordinate of a point to grid(integer) coordinates
                Parameters: x coordinate and y coordinate of point
                Return: Returns an array containing grid coordinates
                        1st index - x coordinate
                        2nd index - y coordinate
            */
            int* convertWorldToGrid(double, double);
            
            /*
                Converts the GRID coordinate of a point to 1D index
                Parameters: x grid coordinate and y grid coordinate of point
                Return: Returns the index of the point
            */
            int coordsToIndex(int, int);

            /*
                Converts the grid coordinate of a point to world coordinates
                Parameters: x grid coordinate and y grid coordinate of point
                Return: Returns an array containing world coordinates
                        1st index - x coordinate
                        2nd index - y coordinate
            */
            double* convertGridToWorld(int, int);
            
            /*
                Converts the 1D index of a point to grid(integer) coordinates
                Parameters: 1D index of point
                Return: Returns an array containing grid coordinates
                        1st index - x coordinate
                        2nd index - y coordinate
            */
            int* indexToCoords(int);

            private:
                // boolean to store if plugin is initialzed
                bool initialized;

                costmap_2d::Costmap2DROS* costmap_ROS;
                costmap_2d::Costmap2D* costmap;
                
                float origin_x;
                float origin_y;
                
                int costmap_width;
                int costmap_height;
                float resolution;
                
                ros::ServiceClient make_plan_srv;
                ros::Publisher plan_publisher;
                
                // for offsetting the position to centre of a grid from side of a grid
                double node_centre_offset;
    };
}

#endif