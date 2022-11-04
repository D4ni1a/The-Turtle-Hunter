#include <ros/ros.h>
#include <math.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <unistd.h>


turtlesim::Pose turtle_catcher;
turtlesim::Pose turtle_runner;
geometry_msgs::Twist turtle_catcher_twist;


double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;
    
    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();

    e.x = setpoint_pose.x - turtle_pose.x;
    e.y = setpoint_pose.y - turtle_pose.y;
    double error = sqrt(pow(e.x,2) + pow(e.y,2));

    derror = error - prev_error;    
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    prev_error = error;
    return u;
}
double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;

    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();
    
    double error = atan2(setpoint_pose.y - turtle_pose.y, setpoint_pose.x - turtle_pose.x) - turtle_pose.theta;
    derror = error - prev_error;
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    
    prev_error = error;
    
    u = u >  4*M_PI? 4*M_PI:u;
    u = u < -4*M_PI?-4*M_PI:u;
    return u;
}
void killTurtle(std::string name, turtlesim::Kill srv, ros::ServiceClient client){
        srv.request.name = name;
        ros::service::waitForService("/kill",ros::Duration(5));
        client.call(srv);
}
ros::Publisher pubCatcher;
ros::Publisher pubStatus;
std_msgs::Bool status_msg;
bool moveTo(turtlesim::Pose finish_pose, double tolerance, ros::ServiceClient kill_client) {
    
    geometry_msgs::Twist cmd_vel_msg;

    std::vector<double> KpKiKd_distance{1.01, 0.2, 0.001};
    std::vector<double> KpKiKd_angle{3.5, 0.05, 0.05};        
    double d = PID_distance(KpKiKd_distance, finish_pose, turtle_catcher, 1.0/10);
    cmd_vel_msg.linear.x = d/2.5; 
    cmd_vel_msg.angular.z = PID_angle(KpKiKd_angle, finish_pose, turtle_catcher, 1.0/10);
    pubCatcher.publish(cmd_vel_msg);
    double dist = sqrt(pow((turtle_catcher.x-finish_pose.x),2.0)+pow((turtle_catcher.y-finish_pose.y),2.0));

    if (dist!=0 && dist<1){
        turtlesim::Kill kill_srv;
        killTurtle("turtle1", kill_srv, kill_client);
        status_msg.data = true;
        return true;
    }
    return false;
}

void catcher_callback(const turtlesim::Pose::ConstPtr& msg){
    turtle_catcher.angular_velocity = msg->angular_velocity;
    turtle_catcher.linear_velocity = msg->linear_velocity;
    turtle_catcher.theta = msg->theta;
    turtle_catcher.x = msg->x;
    turtle_catcher.y = msg->y;
}
void runner_callback(const turtlesim::Pose::ConstPtr& msg){
    turtle_runner.angular_velocity = msg->angular_velocity;
    turtle_runner.linear_velocity = msg->linear_velocity;
    turtle_runner.theta = msg->theta;
    turtle_runner.x = msg->x;
    turtle_runner.y = msg->y;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "catcher");
    ros::NodeHandle n;

    ros::Subscriber subCatcher = n.subscribe("/hunter/pose", 1000, catcher_callback);
    ros::Subscriber subRunner = n.subscribe("/turtle1/pose", 1000, runner_callback);
    ros::ServiceClient kill_client = n.serviceClient<turtlesim::Kill>("/kill");

    pubCatcher = n.advertise<geometry_msgs::Twist>("hunter/cmd_vel", 1000);

    status_msg.data = false;
    pubStatus = n.advertise<std_msgs::Bool>("/finished_player", 1000);


    ros::Rate loop_rate(10);
    while(ros::ok()){
        bool flag = moveTo(turtle_runner, 0.1, kill_client);
        pubStatus.publish(status_msg);
        if (flag) break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok()){
        pubStatus.publish(status_msg);
        ros::spinOnce();
        loop_rate.sleep();
        sleep(3);
        break;
    }
    return 0;

}
