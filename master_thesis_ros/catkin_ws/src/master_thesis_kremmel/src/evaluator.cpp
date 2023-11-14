#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Evaluator
{
public:
    Evaluator(ros::NodeHandle &nTemp) : n(nTemp)
    {
        groundTruthSubscriber.subscribe(n, "/ground_truth/state", 1);
        wheelOdometrySubscriber.subscribe(n, "/ekf_slam/wheel_odometry", 1);
        ekfOdometrySubscriber.subscribe(n, "/ekf_slam/current_state", 1);
        kissOdometrySubscriber.subscribe(n, "/kiss/odometry", 1);
        v_w_numLM_meanFsSubscriber.subscribe(n, "/ekf_slam/vel", 1);
        sync.reset(new Sync(SyncRule(10), groundTruthSubscriber, wheelOdometrySubscriber, ekfOdometrySubscriber, v_w_numLM_meanFsSubscriber));
        sync->registerCallback(boost::bind(&Evaluator::callback, this, _1, _2, _3, _4)); // Zuweisen des synchronisierten Callbacks

        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        std::string csvPath = packagePath + "/results/results.csv";

        results.open(csvPath);
        results << std::fixed; // avoid scientific notation of values

        ros::Duration(3).sleep(); // Warten
    }

    ~Evaluator()
    {
        results.close();
    }

    void callback(const nav_msgs::Odometry::ConstPtr &groundTruthMsg, const nav_msgs::Odometry::ConstPtr &wheelOdomMsg,
                  const nav_msgs::Odometry::ConstPtr &ekfOdomMsg, const nav_msgs::Odometry::ConstPtr &v_w_numLM_meanFsMsg)
    {
        results << v_w_numLM_meanFsMsg->twist.twist.linear.x << "," << v_w_numLM_meanFsMsg->twist.twist.angular.z << "," << v_w_numLM_meanFsMsg->pose.pose.position.x << "," << v_w_numLM_meanFsMsg->pose.pose.position.y
                << "," << ekfOdomMsg->pose.pose.position.x << "," << ekfOdomMsg->pose.pose.position.y << "," << constrain_angle(getYaw(ekfOdomMsg))
                << "," << wheelOdomMsg->pose.pose.position.x << "," << wheelOdomMsg->pose.pose.position.y << "," << constrain_angle(getYaw(wheelOdomMsg))
                << "," << groundTruthMsg->pose.pose.position.x << "," << groundTruthMsg->pose.pose.position.y << "," << constrain_angle(getYaw(groundTruthMsg))
                << "\n";
    }

    double constrain_angle(double radian)
    {
        if (radian < -M_PI)
        {
            radian += 2 * M_PI;
        }
        else if (radian > M_PI)
        {
            radian -= 2 * M_PI;
        }
        return radian;
    }

    double getYaw(const nav_msgs::OdometryConstPtr odomMsg)
    {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

private:
    ros::NodeHandle n;

    message_filters::Subscriber<nav_msgs::Odometry> groundTruthSubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> wheelOdometrySubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> ekfOdometrySubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> kissOdometrySubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> v_w_numLM_meanFsSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> SyncRule;
    typedef message_filters::Synchronizer<SyncRule> Sync;
    boost::shared_ptr<Sync> sync;

    std::ofstream results;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    Evaluator Node(n);

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}