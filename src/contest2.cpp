#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>
#include <string>


float goal_check(nav_msgs::GetPlan &srv, ros::ServiceClient &check_path, float rx, float ry,
                 geometry_msgs::Quaternion quat_start, float gx, float gy, geometry_msgs::Quaternion quat_end) {
    // Checks if goal pose and orientation is possible
    geometry_msgs::PoseStamped Start;
    Start.header.frame_id = "map";
    Start.pose.position.x = rx;
    Start.pose.position.y = ry;
    Start.pose.position.z = 0.0;
    Start.pose.orientation.x = 0.0;
    Start.pose.orientation.y = 0.0;
    Start.pose.orientation.z = quat_start.z;
    Start.pose.orientation.w = quat_start.w;

    geometry_msgs::PoseStamped Goal;
    Goal.header.frame_id = "map";
    Goal.pose.position.x = gx;
    Goal.pose.position.y = gy;
    Goal.pose.position.z = 0.0;
    Goal.pose.orientation.x = 0.0;
    Goal.pose.orientation.y = 0.0;
    Goal.pose.orientation.z = quat_end.z;
    Goal.pose.orientation.w = quat_end.w;

    srv.request.start = Start;
    srv.request.goal = Goal;
    srv.request.tolerance = 0;
    check_path.call(srv);
    return srv.response.plan.poses.size();
}

int main(int argc, char **argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0, 0, 0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes;
    if (!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for (int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: "
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    // Execute strategy.

    // Plan optimal path

    std::cout << "Initializing goals..." << std::endl;
    // First, setup goal poses using boxes.coords
    float goals[11][3];

    //ros spin to get an initial robot pose
    ros::spinOnce();
    ros::Duration(0.10).sleep();

    //Set starting position  
    goals[0][0] = robotPose.x;
    goals[0][1] = robotPose.y;
    goals[0][2] = robotPose.phi;

    float offset = 0.35;
    const double pi = 3.14159;

    // store goal locations based on box coordinates
    float x, y, phi;
    for (int i = 1; i < 11; i++) {
        x = boxes.coords[i - 1][0];
        y = boxes.coords[i - 1][1];
        phi = boxes.coords[i - 1][2];

        goals[i][0] = x + offset * cos(phi);
        goals[i][1] = y + offset * sin(phi);
        goals[i][2] = phi + pi; //so that robot faces image
    }

    //Set up adjacency matrix for optimizing path planning
    float box_adj[11][11];
    float x_one, y_one, x_two, y_two;
    for (int i = 0; i < 11; i++) {
        for (int j = 0; j < 11; j++) {
            x_one = goals[i][0];
            y_one = goals[i][1];
            x_two = goals[j][0];
            y_two = goals[j][1];
            box_adj[i][j] = sqrt(pow((x_one - x_two), 2) + pow((y_one - y_two), 2));

        }
    }

    // Calculate optimized order of visiting boxes
    int order[11];
    int visited[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int cur_vertex = 0;
    int min_dist, min_index;
    for (int i = 0; i < 10; i++) {
        min_dist = 1000;
        min_index = 0;
        for (int j = 0; j < 11; j++) {
            if ((cur_vertex != j) && (visited[j] == 0) && box_adj[cur_vertex][j] < min_dist) {
                min_dist = box_adj[cur_vertex][j];
                min_index = j;
            }
        }
        cur_vertex = min_index;
        order[i] = min_index;
        visited[min_index] = 1;
    }

    // Return to starting point at end
    order[10] = 0;


    int goal_index = 0;
    int tags_found_index;
    int state = 0;
    int tags_found[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool found;
    std::string duplicate;
    float x_goal, y_goal, phi_goal, bphi, correct_goal_size, bx, by, sign, inc;

    // Print to file to mark beginning of new run
    std::ofstream myfile("output.txt", std::ios::app);
    myfile << "Starting new run..." << std::endl;
    myfile.close();

    // Print box order to visit
    std::cout << "Box Order: ";
    for (int i = 0; i < 11; i++)
        std::cout << order[i] - 1 << " ";
    std::cout << "\n";

    while (ros::ok()) {
        ros::spinOnce();
        // State = 0 -> Calculate Goal Pose State
        if (state == 0) {

            // Check if goal location is valid
            offset = 0.35;
            x_goal = goals[order[goal_index]][0];
            y_goal = goals[order[goal_index]][1];
            phi_goal = goals[order[goal_index]][2];

            geometry_msgs::Quaternion w_start = tf::createQuaternionMsgFromYaw(robotPose.phi);
            geometry_msgs::Quaternion w_goal = tf::createQuaternionMsgFromYaw(phi_goal);

            ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan", true);
            nav_msgs::GetPlan srv;

            std::cout << "Checking if plan is valid..." << std::endl;
            correct_goal_size = goal_check(srv, check_path, robotPose.x, robotPose.y, w_start, x_goal, y_goal, w_goal);

            // If goal index == 10, we have finished with all the boxes and are returning to initial pose
            if (goal_index == 10) {
                state = 3;
                continue;
            }

            bx = boxes.coords[order[goal_index] - 1][0];
            by = boxes.coords[order[goal_index] - 1][1];
            bphi = boxes.coords[order[goal_index] - 1][2];

            sign = 1;
            inc = pi / 24;
            // If correct_goal_size <= 0, then plan is not valid and use adjust x, y, phi until feasible goal is found
            while (correct_goal_size <= 0) {
                if (offset < 0.05) {
                    break;
                }

                // In incremental angle is too large, make offset less and restart search
                if (inc > 5 * pi / 12 + 0.01) {
                    offset = offset - 0.01;
                    inc = 0;
                }

                // Calculate new goal and check if feasible
                phi_goal = bphi + pi + sign * inc;
                x_goal = bx + offset * cos(phi_goal - pi);
                y_goal = by + offset * sin(phi_goal - pi);

                w_start = tf::createQuaternionMsgFromYaw(robotPose.phi);
                w_goal = tf::createQuaternionMsgFromYaw(phi_goal);
                correct_goal_size = goal_check(srv, check_path, robotPose.x, robotPose.y, w_start, x_goal, y_goal,
                                               w_goal);

                // flip sign to check left and right side of increment. Add to increment every other loop
                sign = -sign;
                if (sign > 0) {
                    inc = inc + pi / 24;
                }
            }

            goal_index += 1;
            state = 1;
            continue;
        }

        // State = 1 ->Move to Goal State
        if (state == 1) {

            std::cout << "Moving to box " << order[goal_index - 1] - 1 << std::endl;
            found = Navigation::moveToGoal(x_goal, y_goal, phi_goal);

            if (found == false) {
                myfile << "Tag: Could not get path to box. Box #" << order[goal_index - 1] - 1 << " Location:"
                       << "  x: " << x << "  y: "
                       << y << "  phi: " << phi << "\n";
                state = 0;
                continue;

            }

            state = 2;
            continue;
        }

        // State = 2 -> Image Detection State
        if (state == 2) {
            int tag_id = -1;
            std::ofstream myfile("output.txt", std::ios::app);

            // Find tag_id
            tag_id = imagePipeline.getTemplateID(boxes);
            std::cout << "Printing tag_" << tag_id << " to file..." << std::endl;

            // Find coordinates of box found
            x = boxes.coords[order[goal_index - 1] - 1][0];
            y = boxes.coords[order[goal_index - 1] - 1][1];
            phi = boxes.coords[order[goal_index - 1] - 1][2];


            // Check if image is duplicate
            if (tag_id > 0) {
                tags_found_index = tag_id - 1;
            } else {
                tags_found_index = 15;
            }

            if (tags_found[tags_found_index] > 0) {
                duplicate = "True";
            } else {
                duplicate = "False";
            }
            tags_found[tags_found_index] = tags_found[tags_found_index] + 1;

            // Print results to file
            if (tag_id == -1) {
                myfile << "Box #" << order[goal_index - 1] - 1 << "-- Tag: No Tag. Location:" << "  x: " << x << "  y: "
                       << y << "  phi: " << phi << " Duplicate?: " << duplicate << std::endl;
            } else {
                myfile << "Box #" << order[goal_index - 1] - 1 << "-- Tag: " << tag_id << " Location:" << "  x: " << x
                       << "  y: "
                       << y << "  phi: " << phi << " Duplicate?: " << duplicate << std::endl;
            }
            myfile.close();
            state = 0;

        }

        // State = 3 -> Return to Initial Pose State
        if (state == 3) {
            std::cout << "Returning to start point..." << std::endl;
            found = Navigation::moveToGoal(x_goal, y_goal, phi_goal);
            std::cout << "Returned to start! Operation Complete." << std::endl;
            return 0;
        }

    }

    ros::Duration(0.01).

            sleep();

    return 0;
}
