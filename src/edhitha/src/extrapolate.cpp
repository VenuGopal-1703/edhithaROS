#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher imu_extrapolated_pub;
ros::Publisher gps_extrapolated_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

    ROS_INFO("IMU callback triggered");
    // Process IMU data
    // Example: Print input IMU data
    ROS_INFO("IMU Input - Orientation: %.2f, %.2f, %.2f, Angular Velocity: %.2f, %.2f, %.2f",
             msg->orientation.x, msg->orientation.y, msg->orientation.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // Perform extrapolation (linear for demonstration)
    sensor_msgs::Imu imu_extrapolated = *msg;  // Start with input data

    // Extrapolate 5 times
    for (int i = 1; i <= 5; ++i) {
        imu_extrapolated.header.stamp += ros::Duration(0.01);  // Increment timestamp (adjust as needed)
        imu_extrapolated.angular_velocity.x *= 1.1;  // Example extrapolation: Increase angular velocity
        imu_extrapolated.angular_velocity.y *= 1.1;
        imu_extrapolated.angular_velocity.z *= 1.1;
        
        // Publish extrapolated data
        imu_extrapolated_pub.publish(imu_extrapolated);

        // Print extrapolated data (for debug/info)
        ROS_INFO("IMU Extrapolated Data %d - Angular Velocity: %.2f, %.2f, %.2f",
                 i, imu_extrapolated.angular_velocity.x,
                 imu_extrapolated.angular_velocity.y,
                 imu_extrapolated.angular_velocity.z);
    }
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
     ROS_INFO("GPS callback triggered ");
    // Process GPS data
    // Example: Print input GPS data
    ROS_INFO("GPS Input - Latitude: %.6f, Longitude: %.6f, Altitude: %.2f",
             msg->latitude, msg->longitude, msg->altitude);

    // Perform extrapolation (linear for demonstration)
    sensor_msgs::NavSatFix gps_extrapolated = *msg;  // Start with input data

    // Extrapolate 5 times
    for (int i = 1; i <= 5; ++i) {
        gps_extrapolated.header.stamp += ros::Duration(0.2);  // Increment timestamp (adjust as needed)
        gps_extrapolated.latitude += 0.0001;  // Example extrapolation: Increase latitude
        gps_extrapolated.longitude += 0.0001; // and longitude

        // Publish extrapolated data
        gps_extrapolated_pub.publish(gps_extrapolated);

        // Print extrapolated data (for debug/info)
        ROS_INFO("GPS Extrapolated Data %d - Latitude: %.6f, Longitude: %.6f",
                 i, gps_extrapolated.latitude, gps_extrapolated.longitude);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "extrapolate");
    ros::NodeHandle nh;

    // Publishers for extrapolated data
    imu_extrapolated_pub = nh.advertise<sensor_msgs::Imu>("imu/extrapolated", 10);
    gps_extrapolated_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/extrapolated", 10);

    // Subscribe to IMU and GPS topics
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 10, imuCallback);
    ros::Subscriber gps_sub = nh.subscribe("/mavros/global_position/global", 10, gpsCallback);

    // Log ROS info
    ROS_INFO("Getting input data from FCU");

    ros::spin();

    return 0;
}