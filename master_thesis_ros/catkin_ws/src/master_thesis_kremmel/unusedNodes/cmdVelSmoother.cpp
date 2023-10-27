#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class CmdVelSmoother
{
public:
    CmdVelSmoother(ros::NodeHandle &nTemp) : n(nTemp)
    {
        // Publisher und Subscriber initialisieren
        CmdVelSub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &CmdVelSmoother::callback, this);
        CmdVelPub = n.advertise<geometry_msgs::Twist>("/taurob_tracker/cmd_vel_raw", 1);

        last_v = 0;
        last_w = 0;
    }

    // ToDo: Abort callback function when new message arrives. But how???
    void callback(const geometry_msgs::TwistConstPtr &cmd_vel)
    {
        geometry_msgs::Twist smoothed_cmd_vel;

        while (abs(cmd_vel->linear.x - last_v) > 0.05 || abs(cmd_vel->angular.z - last_w) > 0.03)
        {
            if (abs(cmd_vel->linear.x - last_v) < 0.05)
            {
                last_v = cmd_vel->linear.x;
            }
            else
            {
                if (cmd_vel->linear.x - last_v >= 0)
                {
                    std::cout << "going to increase speed from " << last_v << " to " << cmd_vel->linear.x << std::endl;
                    last_v += 0.04;
                }
                else
                {
                    std::cout << "going to decrease speed from " << last_v << " to " << cmd_vel->linear.x << std::endl;
                    last_v -= 0.04;
                }
            }

            if (abs(cmd_vel->angular.z - last_w) < 0.03)
            {
                last_w = cmd_vel->angular.z;
            }
            else
            {
                if (cmd_vel->angular.z - last_w >= 0)
                {
                    std::cout << "going to increase angle speed from " << last_w << " to " << cmd_vel->angular.z << std::endl;
                    last_w += 0.03;
                }
                else
                {
                    std::cout << "going to decrease angle speed from " << last_w << " to " << cmd_vel->angular.z << std::endl;
                    last_w -= 0.03;
                }
            }

            smoothed_cmd_vel.linear.x = last_v;
            smoothed_cmd_vel.angular.z = last_w;
            CmdVelPub.publish(smoothed_cmd_vel);
            ros::Duration(0.06).sleep();
        }
        std::cout << "setting target speeds " << std::endl;
        last_v = cmd_vel->linear.x;
        last_w = cmd_vel->angular.z;
        smoothed_cmd_vel.linear.x = last_v;
        smoothed_cmd_vel.angular.z = last_w;
        CmdVelPub.publish(smoothed_cmd_vel);
    }

private:
    ros::NodeHandle n;
    ros::Subscriber CmdVelSub;
    ros::Publisher CmdVelPub;

    double last_v;
    double last_w;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel smoother");
    ros::NodeHandle n;
    CmdVelSmoother Node(n);
    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}