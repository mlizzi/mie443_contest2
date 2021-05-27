#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>


float goal_check(nav_msgs::GetPlan &srv, ros::ServiceClient &check_path, float rx, float ry,
                 geometry_msgs::Quaternion quat_start, float gx, float gy, geometry_msgs::Quaternion quat_end) {


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
    //ROS_INFO("Make plan: %d", (check_path.call(srv) ? 1 : 0));
    //ROS_INFO("Plan size: %d", srv.response.plan.poses.size());
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
    // First, setup goal poses using boxes.coords

    // std::cout << "Initializing goals...";
    std::cout << "Initializing goals..." << std::endl;

    float goals[11][3];// = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};


    // std::cout << "Initializing goals..." << std::endl;

    //ros spin to get an initial robot pose
    ros::spinOnce();
    ros::Duration(0.10).sleep();

    //Set starting position  
    goals[0][0] = robotPose.x;
    goals[0][1] = robotPose.y;
    goals[0][2] = robotPose.phi;


    float offset = 0.35;
    const double pi = 3.14159;

    float x, y, phi;
    for (int i = 1; i < 11; i++) {
        x = boxes.coords[i - 1][0];
        y = boxes.coords[i - 1][1];
        phi = boxes.coords[i - 1][2];
        //std::cout<<x<<" "<<y<<std::endl;
        goals[i][0] = x + offset * cos(phi);
        goals[i][1] = y + offset * sin(phi);
        goals[i][2] = phi + pi; //so that robot faces image
        //std::cout<<goals[i][0]<<" "<<goals[i][1]<<std::endl;
    }

    float box_adj[11][11];
    //float pos_list[11] = [0] * 11;
    //Set up adjacency matrix
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

//    std::cout << x_one;
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

    std::vector<int> tags_found;
    int goal_index = 0;
    int tag_id;
    int state = 0;
    bool found;
    char duplicate;
    float x_goal, y_goal, phi_goal, bphi, correct_goal, bx, by, sign, inc;

    //std::cout << "Starting while loop..." << std::endl;
    std::ofstream myfile("output.txt", std::ios::app);
    myfile << "Starting new run..." << std::endl;
    myfile.close();

    std::cout << "Box Order: ";
    for (int i = 0; i < 11; i++)
        std::cout << order[i] - 1 << " ";
    std::cout << "\n";



    while (ros::ok()) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        //std::cout <<"State: " << state << std::endl;

        if (state == 0) {
            x_goal = goals[order[goal_index]][0];
            y_goal = goals[order[goal_index]][1];
            phi_goal = goals[order[goal_index]][2];

            geometry_msgs::Quaternion w_start = tf::createQuaternionMsgFromYaw(robotPose.phi);
            geometry_msgs::Quaternion w_goal = tf::createQuaternionMsgFromYaw(phi_goal);

            ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan", true);
            nav_msgs::GetPlan srv;

            std::cout << "Checking if plan is valid..." << std::endl;
            correct_goal = goal_check(srv, check_path, robotPose.x, robotPose.y, w_start, x_goal, y_goal, w_goal);

            // If we have visited all the boxes, proceed to Return Start Location state
            if (goal_index == 10) {
                state = 3;
                continue;
            }

            bx = boxes.coords[order[goal_index] - 1][0];
            by = boxes.coords[order[goal_index] - 1][1];
            bphi = boxes.coords[order[goal_index] - 1][2];

            sign = 1;
            inc = pi / 12;
            offset = 0.35;

            // If correct_goal is not valid, adjust goal location slightly until a valid location is found
            while (correct_goal <= 0) {
                if (offset < 0.05) {
                    found = false;
                    break;
                }

                if (inc > 5 * pi / 6 + 0.01) {
                    offset = offset - 0.01;
                    inc = 0;
                }
                //std::cout << "Goal Failed, looking for new goal" <<std::endl;
                //std::cout << x_goal << " " << y_goal << " " << phi_goal << " " << sign << " " << " " << offset << std::endl;
                phi_goal = bphi + pi + sign * inc;
                x_goal = bx + offset * cos(phi_goal - pi);
                y_goal = by + offset * sin(phi_goal - pi);

                w_start = tf::createQuaternionMsgFromYaw(robotPose.phi);
                w_goal = tf::createQuaternionMsgFromYaw(phi_goal);
                correct_goal = goal_check(srv, check_path, robotPose.x, robotPose.y, w_start, x_goal, y_goal, w_goal);
                //std::cout << "New path generated has this many steps: " <<correct_goal<<". Trying again..."<<std::endl;

                sign = -sign;
                if (sign > 0) {
                    inc = inc + pi / 12;
                }

            }

            found = true;
            goal_index += 1;
            state = 1; //Go to navigation state
            continue;
        }
        if (state == 1) {
            std::cout << "Moving to box " << order[goal_index - 1] - 1
                      << std::endl;//<< "--> x_goal: " << x_goal << "  y_goal: " << y_goal << "  phi_goal: " << phi_goal << std::endl;
            if (found == false) {
                myfile << "Box #" << order[goal_index - 1] - 1 << "Tag: Could not get path to box. Location:"
                       << "  x: " << x << "  y: " << y << "  phi: " << phi << std::endl;
                state = 0;
                continue;

            }
            Navigation::moveToGoal(x_goal, y_goal, phi_goal);
            state = 2;
            continue;
        }


        if (state == 2) { //image detection
            int tag_id = -1;
            tag_id = imagePipeline.getTemplateID(boxes);
            std::ofstream myfile("output.txt", std::ios::app);
            x = boxes.coords[order[goal_index - 1] - 1][0];
            y = boxes.coords[order[goal_index - 1] - 1][1];
            phi = boxes.coords[order[goal_index - 1] - 1][2];

            for (int i = 0; i < tags_found.size(); i++) {
                if (tag_id == tags_found[i]) {
                    duplicate = 'T';
                } else {
                    duplicate = 'F';
                }
                tags_found.push_back(tag_id);
            }

            if (tag_id == -1) {
                myfile << "Box #" << order[goal_index - 1] - 1 << " -- Tag: No Tag. Location:" << "  x: " << x
                       << "  y: "
                       << y << "  phi: " << phi << " Duplicate: " << duplicate << std::endl;
            } else {
                myfile << "Box #" << order[goal_index - 1] - 1 << " -- Tag: tag_" << tag_id << ".jpg. Location:"
                       << "  x: " << x << "  y: " << y << "  phi: " << phi << " Duplicate: " << duplicate << std::endl;
            }
            myfile.close();
            state = 0;
        }

    }

    if (state == 3) {
        std::cout << "Returning to start point..." << std::endl;
        found = Navigation::moveToGoal(x_goal, y_goal, phi_goal);
        std::cout << "Returned to start! Operation Complete." << std::endl;
        return 0;
    }

    ros::Duration(0.01).sleep();
    return 0;
}
