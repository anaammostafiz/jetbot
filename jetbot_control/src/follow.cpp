#include "jetbot_vision/ObjDepths.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

class Follower{
private:
    // Communicate with nodes
    ros::NodeHandle n;
    // Depths sub
    ros::Subscriber depths_sub;
    std::string depths_topic;
    void depths_callback(const jetbot_vision::ObjDepths::ConstPtr &depths_msg);
    // Velocity pub
    ros::Publisher vel_pub;
    std::string vel_topic;
    geometry_msgs::Twist vel_msg;

    // Class parameters
    const double target_x;
    const double x_thresh;
    const double target_z;
    const double z_thresh;
    const std::string leader;

    std::vector<std::string> classes;
    std::vector<double> depths;
    std::vector<double> centers_x;

public:
    Follower(const double x, const double xth, const double z, const double zth, const std::string L);
    void follow_leader();
};

Follower::Follower(const double x, const double xth, const double z, const double zth, const std::string L)
    : target_x(x), x_thresh(xth), target_z(z), z_thresh(zth), leader(L){
    n = ros::NodeHandle("~");
    depths_topic = "/detectnet/obj_depths";
    depths_sub = n.subscribe(depths_topic, 10, &Follower::depths_callback, this);
    vel_topic = "/cmd_vel";
    vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName(vel_topic),1);

    ROS_INFO("Initializing follower .................................");
    usleep(2000000);
    ros::spinOnce();
}

void Follower::depths_callback(const jetbot_vision::ObjDepths::ConstPtr &depths_msg){

    classes = depths_msg->classes;
    depths = depths_msg->depths;
    centers_x = depths_msg->centers_x;

}

void Follower::follow_leader(){
     
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    const double k_z = 0.5;
    const double gs_x[] = {0.001, 0.0005, 0.0};

    if (!classes.empty()) {
        // Check if leader is present in the classes
        auto it = std::find(classes.begin(), classes.end(), leader);
        if (it != classes.end()) {
            int index = std::distance(classes.begin(), it);
        
            double error_z = depths[index] - target_z;
            std::cout << "error_z: " << error_z << std::endl;

            if (depths[index] >= target_z + z_thresh){
                // move forward
                linear_vel = std::min(k_z * error_z, 1.0);
                std::cout << "MOVING FORWARD: " << linear_vel << std::endl;
            }

            if (centers_x[index] <= target_x - x_thresh || centers_x[index] >= target_x + x_thresh){
                double k_x;
                if (error_z >= 0.5){
                    k_x = gs_x[0]; 
                } else if (error_z >= 0.35){
                    k_x = gs_x[1];
                } else {
                    k_x = gs_x[2];
                }
                
                if (centers_x[index] <= target_x - x_thresh){
                    // rotate counterclockwise
                    double error_x = target_x - centers_x[index];    
                    angular_vel = std::min(k_x * error_x, 1.0);
                    std::cout << "ROTATING LEFT: k_x = " << k_x << " angular_vel = " << angular_vel << std::endl;
                } else if (centers_x[index] >= target_x + x_thresh){
                    // rotate clockwise
                    double error_x = target_x - centers_x[index];
                    angular_vel = std::max(k_x * error_x, -1.0);
                    std::cout << "ROTATING RIGHT: k_x = " << k_x << " angular_vel = " << angular_vel << std::endl;
                } 
            }

        }
    }

    // Adjusting the velocity message
    vel_msg.linear.x = linear_vel;
    vel_msg.angular.z = angular_vel;

    // Publish the velocity message
    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "follower");

    // 640x480 image
    Follower pet(320.0, 50.0, 0.7, 0.2, "person");

    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        pet.follow_leader();
        ros::spinOnce();
        loop_rate.sleep();
    } 

    return 0;
}
