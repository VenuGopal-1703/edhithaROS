#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

// Global variable to store the current state
mavros_msgs::State current_state;

// Callback function to update the state
void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    ROS_INFO("[armdisarm_node] Current mode: %s, armed: %s", current_state.mode.c_str(), current_state.armed ? "true" : "false");
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "armdisarm_node");
    ros::NodeHandle nh;

    // Create a service client to arm the drone
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // Subscribe to the /mavros/state topic
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);

    // Create the request and response objects
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // Set to false to disarm

    // Wait for the service to be available
    if (arming_client.waitForExistence(ros::Duration(5.0)))
    {
        ROS_INFO("Service available, attempting to arm the drone.");
        if (arming_client.call(arm_cmd))
        {
            if (arm_cmd.response.success)
                ROS_INFO("Drone armed successfully.");
            else
                ROS_ERROR("Failed to arm the drone.");
        }
        else
        {
            ROS_ERROR("Failed to call service.");
        }
    }
    else
    {
        ROS_ERROR("Service not available.");
    }

    // Spin to keep the node running and receiving state updates
    ros::spin();

    return 0;
}
