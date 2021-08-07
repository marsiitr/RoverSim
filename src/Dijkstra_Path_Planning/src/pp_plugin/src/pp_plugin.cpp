#include <pluginlib/class_list_macros.h>
#include <pp_plugin/pp_plugin.h>
#include <nav_msgs/Path.h>

//Macro to register class as plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {
    GlobalPlanner::GlobalPlanner() {
        initialized = false;
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if(!initialized) {
            // Initialize all the required variables and nodes
            ros::NodeHandle node;
            costmap_ROS = costmap_ros;
            costmap = costmap_ros->getCostmap();
            origin_x = costmap->getOriginX();
            origin_y = costmap->getOriginY();
            costmap_height = costmap->getSizeInCellsY();
            costmap_width = costmap->getSizeInCellsX();
            resolution = costmap->getResolution();

            make_plan_srv = node.serviceClient<path_planning_msg::pp_msg>("make_plan");
            make_plan_srv.waitForExistence();

            plan_publisher = node.advertise<nav_msgs::Path>("plan", 1);
            node_centre_offset = resolution/2;
            initialized = true;
        }
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        int start_idx, goal_idx;
        double start_x_grid, start_y_grid, goal_x_grid, goal_y_grid;

        plan.clear();
        
        // Convert the costmap to 1D to be used with algorithm
        std::vector<int> costmap_1d;
        for(int x=0; x<costmap_width; x++) {
            for(int y=0; y<costmap_height; y++) {
                costmap_1d.push_back(costmap->getCost(x,y));
            }
        }

        double start_y_world = start.pose.position.y;
        double start_x_world = start.pose.position.x;
        double goal_x_world = goal.pose.position.x;
        double goal_y_world = goal.pose.position.y;
            
        // Convert start and goal coordinates to grid coordinates
        if(isPositionValid(start_x_world, start_y_world) && isPositionValid(goal_x_world, goal_y_world)) {
            int* temp;
            temp = convertWorldToGrid(start_x_world,start_y_world);
            start_x_grid = temp[0];
            start_y_grid = temp[1];
            temp = convertWorldToGrid(goal_x_world,goal_y_world);
            goal_x_grid = temp[0];
            goal_y_grid = temp[1];
            start_idx = coordsToIndex(start_x_grid, start_y_grid);
            goal_idx = coordsToIndex(goal_x_grid, goal_y_grid);
            delete temp;
        }
        else {
            ROS_WARN("Start or goal not in bounds");
            return false;
        }

        // Call the path planning server to get path
        path_planning_msg::pp_msg make_plan;
        make_plan.request.costmap = costmap_1d;
        make_plan.request.goal = goal_idx;
        make_plan.request.start = start_idx;
        make_plan.request.height = costmap_height;
        make_plan.request.width = costmap_width;

        make_plan_srv.call(make_plan);

        std::vector<int> path_1d = make_plan.response.plan;

        if(path_1d.size() != 0) {
            // Convert all points in the path from index to coordinates and use
            // convert all coords to PoseStamped to be returned to move_base
            for(int i=0; i<path_1d.size(); i++) {
                int x_grid, y_grid;
                double x_world, y_world;
                int* temp_int;
                double* temp_dbl;
                temp_int = indexToCoords(path_1d[i]);
                x_grid = temp_int[0];
                y_grid = temp_int[1];
                temp_dbl = convertGridToWorld(x_grid, y_grid);
                x_world = temp_dbl[0];
                y_world = temp_int[1];
                delete temp_int;
                delete temp_dbl;

                geometry_msgs::PoseStamped pos;
                pos.pose.position.x = x_world;
                pos.pose.position.y = y_world;
                pos.pose.orientation.x = 0;
                pos.pose.orientation.y = 0;
                pos.pose.orientation.z = 0;
                pos.pose.orientation.w = 1;

                plan.push_back(pos);
            }

            // Publish the path
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "map";
            gui_path.header.stamp = ros::Time::now();
            for(int i=0; i<plan.size(); i++) {
                gui_path.poses.push_back(plan[i]);
            }
            plan_publisher.publish(gui_path);
            return true;
        }
        else {
            return false;
        }
    }

    bool GlobalPlanner::isPositionValid(double x_world, double y_world) {
        if(x_world<origin_x || y_world<origin_y || x_world>(costmap_width*resolution)+origin_x || y_world>(costmap_height*resolution)+origin_y) {
            return false;
        }
        else {
            return true;
        }
    }

    int* GlobalPlanner::convertWorldToGrid(double x_world, double y_world) {
        int* coords_grid;
        coords_grid = new int[2];
        coords_grid[0] = floor((x_world - origin_x)/resolution);
        coords_grid[1] = floor((y_world - origin_y)/resolution);
        return coords_grid;
    }

    int GlobalPlanner::coordsToIndex(int x_grid, int y_grid) {
        return (y_grid*costmap_width) + x_grid;
    }

    double* GlobalPlanner::convertGridToWorld(int x_grid, int y_grid) {
        double* coords_world;
        coords_world = new double[2];
        coords_world[0] = (x_grid*resolution) + origin_x + node_centre_offset;
        coords_world[1] = (y_grid*resolution) + origin_y + node_centre_offset;
        return coords_world;
    }

    int* GlobalPlanner::indexToCoords(int idx) {
        int* coords_grid;
        coords_grid = new int[2];
        coords_grid[0] = idx%costmap_width;
        coords_grid[1] = floor(idx/costmap_width);
        return coords_grid;
    }
};