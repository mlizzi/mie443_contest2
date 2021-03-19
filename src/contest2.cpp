#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
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





	
    float offset = 0.25;
    const double pi = 3.14159;
	
	float x, y, phi;
    for(int i = 1; i<11; i++){
		x = boxes.coords[i-1][0];
		y = boxes.coords[i-1][1];
		phi = boxes.coords[i-1][2];
		//std::cout<<x<<" "<<y<<std::endl;
		goals[i][0] = x + offset * cos(phi);
		goals[i][1] = y + offset * sin(phi);
		goals[i][2] = phi + pi; //so that robot faces image 
		//std::cout<<goals[i][0]<<" "<<goals[i][1]<<std::endl;
    }
	
    float box_adj[11][11];
    //float pos_list[11] = [0] * 11;
	std::cout << x;
    //Set up adjacency matrix
	float x_one, y_one, x_two, y_two;
    for(int i = 0; i<11; i++){
    	for (int j = 0; j<11; j++){
            x_one = goals[i][0];
			y_one = goals[i][1];
			x_two = goals[j][0];
			y_two = goals[j][1];
		    box_adj[i][j] = sqrt(pow((x_one-x_two), 2) + pow((y_one-y_two), 2));

    	}
    }

	std::cout << x_one;
	// Calculate optimized order of visiting boxes
    int order[11];
    int visited[11] = {1,0,0,0,0,0,0,0,0,0,0};
    int cur_vertex = 0;
	int min_dist, min_index;
    for(int i = 0; i<10; i++){
        min_dist = 1000;
        min_index = 0; 
    	for(int j = 0; j<11; j++){
    		if((cur_vertex!=j) && (visited[j] == 0) && box_adj[cur_vertex][j] < min_dist){
		    	min_dist =  box_adj[cur_vertex][j];
		    	min_index = j;
    		}
		}
		cur_vertex = min_index;
		order[i] = min_index;
		visited[min_index] = 1; 
    }
	
	// Return to starting point at end
	order[10] = 0;
	
    
	int goal_index = 1;
	int state = 0;
	float x_goal, y_goal, phi_goal;
	
	std::cout << "Starting while loop..." << std::endl;
	std::cout << "Box Order: ";
	std::ofstream myfile("output.txt", std::ios::app);
	myfile << "Starting new run..." << std::endl;
	myfile.close();

	for (int i = 0; i < 11; i++) 
		std::cout << order[i]-1 << " ";
	std::cout << "\n";
    while(ros::ok()) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
		std::cout <<"State: " << state << std::endl;
	
	
	
	if (state == 0) {
			
            x_goal = goals[order[goal_index]][0];
			y_goal = goals[order[goal_index]][1];
			phi_goal = goals[order[goal_index]][2];
            goal_index += 1;
            state = 1; //Go to navigation state
            continue;
        }
        if (state == 1){
			std::cout << "Moving to box " << order[goal_index-1] - 1 <<"--> x_goal: " << x_goal <<"  y_goal: "<< y_goal <<" "<< std::endl; 
            Navigation::moveToGoal(x_goal, y_goal, phi_goal);		
			state = 2;
            //does it automatically break out if successful
/*
            if (sqrt((goal[0]-robotPose.x)**2 + (goal[1]-robotPose.y)**2) < 0.1){
                state = 2; //Go to scan state
            }*/
        }


        if (state == 2){ //image detection
			int image_id = -1;
            image_id = imagePipeline.getTemplateID(boxes);
            std::ofstream myfile("output.txt", std::ios::app);
			x = boxes.coords[order[goal_index-1] - 1][0];
			y = boxes.coords[order[goal_index-1] - 1][1];
			phi = boxes.coords[order[goal_index-1] - 1][2];
			std::cout <<"Printing to file";
			if(image_id == -1){
                myfile << "Tag: No Tag. Box #" <<order[goal_index-1] - 1<<" Location:" << "  x: " << x << "  y: " << y << "  phi: " << phi << "\n"; // TODO: Need location
            }
            else{
                myfile << "Tag: tag_" << image_id << ".jpg. Location:" << "  x: " << x << "  y: " << y << "  phi: " << phi << "\n"; // TODO: Need location
            }
			std::cout<<image_id<<std::endl;
            myfile.close();
            state = 0;
			
        }

        ros::Duration(0.01).sleep();
    }
    return 0;
}
