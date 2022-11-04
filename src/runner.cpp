#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <cstdlib>



turtlesim::Pose pose1;
turtlesim::Pose posePrey;
std_msgs::Bool status;

float random_float(float max){
    return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/max));
}

int random_color(){
    return abs(static_cast <int> (rand()) / 255);
}


class Turtles{
    public:
        Turtles(ros::NodeHandle nH){
            n = nH;
            spawnClient = n.serviceClient<turtlesim::Spawn>("/spawn");
            killerClient = n.serviceClient<turtlesim::Kill>("/kill");
        }
        void kill(std::string name) {
            turtlesim::Kill srv;
            srv.request.name = name;
            ros::service::waitForService("/kill", ros::Duration(5));
            killerClient.call(srv);
        }
        void spawn(std::string name){
            spawnTurtle(setPose(name), spawnClient);
        }

        void spawn(std::string name, float x, float y){
            spawnTurtle(setPose(name, x, y), spawnClient);
        }
        void changePen(std::string name, int r, int g, int b, int width){
              turtlesim::SetPen::Request req;
              req.r = r;
              req.g = g;
              req.b = b;
              req.width = width;
              penSettings(req, name);
        }
        void setOffPen(std::string name){
              turtlesim::SetPen::Request req;
              req.off = 1;
              penSettings(req, name);
        }

    private:
        ros::NodeHandle n;
        ros::ServiceClient spawnClient;
        ros::ServiceClient killerClient;

        turtlesim::Spawn::Request setPose(std::string name){
            turtlesim::Spawn::Request req;
            req.name = name;

            req.x = abs(random_float(10));
            req.y = abs(random_float(10));
            req.theta = random_float(2*M_PI);
            return req;
        }

        turtlesim::Spawn::Request setPose(std::string name, float x, float y){
            turtlesim::Spawn::Request req;
            req.name = name;

            req.x = x;
            req.y = y;
            req.theta = 0.0;
            return req;
        }

        void spawnTurtle(turtlesim::Spawn::Request req, ros::ServiceClient spawnClient) {
            turtlesim::Spawn::Response res;
            spawnClient.call(req,res);
        }

        void penSettings(turtlesim::SetPen::Request reqPen, std::string name){
              ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/"+name+"/set_pen");
              turtlesim::SetPen::Response resPen;
              pen.call(reqPen, resPen);

        }
};

void callbackPose1(const turtlesim::Pose::ConstPtr& msg){
    pose1 = *msg;
    // ROS_INFO("Added info about pose1");
}
void callbackPose2(const turtlesim::Pose::ConstPtr& msg){
    posePrey = *msg;
//     ROS_INFO("Added info about posePrey");
}
void callbackPose3(const std_msgs::Bool::ConstPtr& msg){
    status = *msg;
//     ROS_INFO("Added info about posePrey");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "runner");
    ros::NodeHandle n;
    srand (static_cast <unsigned> (time(0)));

    Turtles mainTurtle(n);

    mainTurtle.kill("turtle1");

    mainTurtle.spawn("prey");
    mainTurtle.spawn("turtle1", 5.5, 5.5);
    mainTurtle.changePen("turtle1", random_color(), random_color(), random_color(), 0);
    mainTurtle.spawn("hunter", 0.0, 0.0);
    mainTurtle.setOffPen("hunter");

//     ros::param::background_g = 100;

//     ros::param("/background_g",100);
//     ros::param("/background_b",0);


    ros::Subscriber subPlayer = n.subscribe("/turtle1/pose", 1000, callbackPose1);
    ros::Subscriber subPrey = n.subscribe("/prey/pose", 1000, callbackPose2);
    ros::Subscriber subStatus = n.subscribe("/finished_player", 1000, callbackPose3);

    ros::Rate rate(3.7);
    int count = 0;
    while (ros::ok()){
        double dist = sqrt(pow((pose1.x-posePrey.x),2.0)+pow((pose1.y-posePrey.y),2.0));
        if (status.data){
              break;
        }
        else if (dist!=0 && dist<0.8){
            mainTurtle.kill("prey");
            count++;
            ROS_INFO_STREAM("Your score is: "<<count);
            dist = 100;
            mainTurtle.spawn("prey");


            mainTurtle.changePen("turtle1", random_color(), random_color(), random_color(), int(count)); 
            sleep(1);
        } 
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("GAME OVER! Your score is: "<<count);
    sleep(5);
    ros::shutdown();
    return 0;
}
