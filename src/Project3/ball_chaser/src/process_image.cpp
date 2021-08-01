#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

enum Order {
    NoOrder,
    Stop,
    Left,
    Right,
    Forward
};

// Define a global client that can request services
ros::ServiceClient client;
Order prevOrder = NoOrder;

// This function calls the command_robot service to drive the robot in the
// specified direction
void drive_robot(float lin_x, float ang_z){
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget service;
    service.request.linear_x = lin_x;
    service.request.angular_z = ang_z;

    // Call the safe_move service and pass the
    // requested joint angles
    if (!client.call(service)) {
        ROS_ERROR("Failed to call service drive_bot");
    }
}

void process_image_callback(const sensor_msgs::Image img) {
    int white_pixel = 255;
    int ballPosition = -1;
    bool ballFound = false;

    for (int i = 0; i < img.height * img.step; i+=3) {
        int pixel_x = i % img.step;
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
            ballFound = true;
            ballPosition = pixel_x;
            break;
        }
    }

    // Calculate ball middle
    const int middle = ballPosition;

    // if there is white not ball
    if (!ballFound) {
        if (prevOrder != Stop) {
             prevOrder = Stop;
            drive_robot(0, 0);
        }
    } else if (middle < img.step * 0.3) {
        // go left
        if(prevOrder != Left){
            prevOrder = Left;
            drive_robot(0, 0.5);
            ROS_INFO_STREAM("Moving to the left! " << middle);
        }
    } else if (middle > img.step * 0.7) {
        // go right
        if(prevOrder != Right){
            prevOrder = Right;
            drive_robot(0, -0.5);
            ROS_INFO_STREAM("Moving to the right! " << middle);
        }
        
    } else if(prevOrder != Forward){
        // go forward
        prevOrder = Forward;
        drive_robot(0.5, 0);
        ROS_INFO_STREAM("Moving forward " << middle);
    }
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the
    // process_image_callback function
    ros::Subscriber sub1 =
        n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}