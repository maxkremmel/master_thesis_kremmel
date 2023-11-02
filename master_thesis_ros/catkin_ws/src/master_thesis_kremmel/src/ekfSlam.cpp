#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include "json.hpp"
#include <master_thesis_kremmel/Landmark.h>
#include <master_thesis_kremmel/MoveRobot.h>

using json = nlohmann::json;

class EKFSlam
{
public:
    EKFSlam(ros::NodeHandle &nTemp) : n(nTemp)
    {
        jointStateSubcriber.subscribe(n, "/joint_states", 10);
        laserScanSubscriber.subscribe(n, "/velodyne_points", 10);

        storedLandmarksPublisher = n.advertise<sensor_msgs::PointCloud2>("ekf_slam/stored_landmarks", 1);
        convergedLandmarksPublisher = n.advertise<sensor_msgs::PointCloud2>("ekf_slam/converged_landmarks", 1);
        covEllipseMsgPublisher = n.advertise<visualization_msgs::Marker>("/ekf_slam/current_state_marker", 1);
        predictedMeasPublisher = n.advertise<visualization_msgs::Marker>("/ekf_slam/predicted_meas", 1);
        currentStatePublisher = n.advertise<nav_msgs::Odometry>("/ekf_slam/current_state", 1);
        wheelOdometryPublisher = n.advertise<nav_msgs::Odometry>("/ekf_slam/wheel_odometry", 1);
        measuredLMposePublisher = n.advertise<visualization_msgs::Marker>("/ekf_slam/measured_lm_pose", 1);
        velocityPublisher = n.advertise<nav_msgs::Odometry>("/ekf_slam/vel", 1); // linear.z is used to publish the number of stored LM

        sync.reset(new Sync(SyncRule(10), jointStateSubcriber, laserScanSubscriber));
        sync->registerCallback(boost::bind(&EKFSlam::predict, this, _1, _2));
        listener = new tf2_ros::TransformListener(tfBuffer); // TransformListener mit Buffer initialisieren

        newLMservice = n.advertiseService("newLMservice", &EKFSlam::newLMcallback, this);
        moveCurrentStateService = n.advertiseService("moveCurrentState", &EKFSlam::moveCurrentStateCallback, this);

        int NUM_LM; // Maximale Anzahl der Landmarken
        // Paramenter aus Parameterserver auslesen
        ros::param::get("/EKFSlam/robotBase", robotBase);
        ros::param::get("/EKFSlam/robotWheelRadius", robotWheelRadius);
        ros::param::get("/EKFSlam/fitnessScoreThreshhold", fitnessScoreThreshhold);
        ros::param::get("/EKFSlam/maxNumLandmarks", NUM_LM);

        // Initialisierung des EKF
        EPS = 1e-4;
        DIMsize = 3 + 2 * NUM_LM; // Dimension der Jakobimatrizen und der Covarianzmatrizen

        alphaRightLast = 0;
        alphaLeftLast = 0;

        state_vector.resize(DIMsize);
        state_vector.setZero();
        ros::param::get("/EKFSlam/x_pos", state_vector(0));
        ros::param::get("/EKFSlam/y_pos", state_vector(1));
        ros::param::get("/EKFSlam/yaw", state_vector(2));
        state_vector_last.resize(DIMsize);
        state_vector_last.setZero();
        state_vector_wheel_odom.setZero();
        state_vector_wheel_odom = state_vector.head(3);

        stored_landmarks = new Landmark[NUM_LM];

        Cov.resize(DIMsize, DIMsize);
        Cov.setZero();
        Eigen::MatrixXd diag(2 * NUM_LM, 2 * NUM_LM);
        diag.setIdentity();
        diag = diag * 100000.0;
        Cov.bottomRightCorner(2 * NUM_LM, 2 * NUM_LM) = diag;
        Cov(0, 0) = 0;
        Cov(1, 1) = 0;
        Cov(2, 2) = 0;

        Q.resize(2, 2);
        Q << 5, 0, // Werte aus Velodyne VLP-16 Datenblatt
            0, 10;

        last_timestamp = ros::Time::now().toSec();

        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        std::string jsonPath = packagePath + "/data/stored_landmarks.json";

        // LM JSON Pfad und Inputstream konfigurieren
        std::ifstream f(jsonPath);
        initStateVectorWithLMs(json::parse(f));

        ros::Duration(2).sleep(); // Warten bis rviz geladen ist

        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_footprint";

        predictedMeas.header.stamp = actualMeas.header.stamp = ros::Time::now();
        predictedMeas.ns = "predicted_measurement";
        predictedMeas.action = visualization_msgs::Marker::ADD;
        predictedMeas.pose.orientation.w = 1.0;
    }

    void predict(const sensor_msgs::JointState::ConstPtr &jointStates, const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
    {
        double dt = ros::Time::now().toSec() - last_timestamp; // Zeitspanne seit letztem Funktionsaufruf berechnen
        last_timestamp = ros::Time::now().toSec();

        Eigen::MatrixXd G; // Jakobi des Bewegungsmodells
        G.resize(DIMsize, DIMsize);
        G.setIdentity();

        double alphaRight = jointStates->position[0] - alphaRightLast;
        double alphaLeft = jointStates->position[1] - alphaLeftLast;
        alphaRightLast = jointStates->position[0];
        alphaLeftLast = jointStates->position[1];

        double vr = (alphaRight * robotWheelRadius) / dt; // Berechnen der Umfangsgeschwindigkeit des rechten Rades
        double vl = (alphaLeft * robotWheelRadius) / dt;  // Berechnen der Umfangsgeschwindigkeit des linken Rades

        double v = (vr + vl) / 2;         // Berechnung der Geschwindigkeit des Roboters in seiner x-Richtung
        double w = (vr - vl) / robotBase; // Berechnung der Winkelgeschwindigkeit um seine z-Achse
        double theta = state_vector_last(2);

        Eigen::Vector3d newMovement;
        newMovement.setZero();

        if (abs(w) > EPS) // Verhindern von Division durch 0 bzw. fast 0
        {
            newMovement(0) = -(v / w) * sin(theta) + (v / w) * sin(theta + w * dt);
            newMovement(1) = +(v / w) * cos(theta) - (v / w) * cos(theta + w * dt);
            newMovement(2) = w * dt;

            G(0, 2) = -(v / w) * cos(theta) + (v / w) * cos(theta + w * dt);
            G(1, 2) = -(v / w) * sin(theta) + (v / w) * sin(theta + w * dt);
        }
        else
        {
            newMovement(0) = cos(theta) * v * dt;
            newMovement(1) = sin(theta) * v * dt;

            // D'Hospital Regel angewendet um numerische Fehler (Division durch Null) zu vermeiden
            G(0, 2) = -v * sin(theta) * dt;
            G(1, 2) = v * cos(theta) * dt;
        }

        state_vector.head(3) += newMovement;
        state_vector_wheel_odom += newMovement;

        nav_msgs::Odometry currentWheelOdometry;
        currentWheelOdometry.pose.pose.position.x = state_vector_wheel_odom(0);
        currentWheelOdometry.pose.pose.position.y = state_vector_wheel_odom(1);
        currentWheelOdometry.pose.pose.orientation = tf2::toMsg(getQuat(state_vector_wheel_odom(2)));
        currentWheelOdometry.child_frame_id = "base_footprint";
        currentWheelOdometry.header.frame_id = "odom";
        currentWheelOdometry.header.stamp = jointStates->header.stamp;
        wheelOdometryPublisher.publish(currentWheelOdometry);

        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
        R(0, 0) = 0.01;
        R(1, 1) = 0.01;
        R(2, 2) = 0.01;
        Cov = G * Cov * G.transpose();
        Cov.block(0, 0, 3, 3) += R;

        numberOfDetectedLandmarks = 0;
        meanFitnessScore = 0;
        correct(new_laserscan_msg);
        meanFitnessScore /= numberOfDetectedLandmarks;

        state_vector_last = state_vector; // Aktuellen Zustandsvektor in state_vector_last zwischenspeichern

        nav_msgs::Odometry velAndNumOfLmMsg;

        velAndNumOfLmMsg.header = jointStates->header;
        velAndNumOfLmMsg.twist.twist.linear.x = v;
        velAndNumOfLmMsg.twist.twist.angular.z = w;
        velAndNumOfLmMsg.pose.pose.position.x = numberOfDetectedLandmarks;
        velAndNumOfLmMsg.pose.pose.position.y = meanFitnessScore;
        velocityPublisher.publish(velAndNumOfLmMsg);

        publishEllipses();
        sendNewBaseLinkTransform();
    }

    void correct(const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*new_laserscan_msg, *pcl_laserscan);

        pcl::PointCloud<pcl::PointXYZ>::Ptr storedLMs(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLMs(new pcl::PointCloud<pcl::PointXYZ>);
        // for-Schleife durch alle gespeicherten LM's
        int cov_index = 0;
        Eigen::MatrixXd H_i;
        H_i.resize(2, DIMsize);
        H_i.setZero();

        predictedMeas.points.clear();
        measuredLMpose.points.clear();

        for (int i = 0; i < numberOfStoredLandmarks; i++)
        {
            cov_index = 3 + 2 * i;
            Eigen::Matrix<double, 2, 1> delta = Eigen::Matrix<double, 2, 1>::Zero();
            delta(0) = state_vector(cov_index) - state_vector(0);
            delta(1) = state_vector(cov_index + 1) - state_vector(1);
            double q = delta.transpose() * delta; // Euklidische Distanz
            // If stored LM is closer than 4 m...
            if (sqrt(q) < 4)
            {
                Eigen::Vector2d predictedMeasurment;
                predictedMeasurment(0) = sqrt(q);
                predictedMeasurment(1) = constrain_angle(atan2(delta(1), delta(0)) - state_vector(2));

                publishPredictedMeas(i);

                Eigen::MatrixXd H_low(2, 5);
                H_low << -sqrt(q) * delta(0), -sqrt(q) * delta(1), 0, sqrt(q) * delta(0), sqrt(q) * delta(1),
                    delta(1), -delta(0), -q, -delta(1), delta(0);
                H_low /= q;

                H_i.block(0, 0, 2, 3) = H_low.block(0, 0, 2, 3);
                H_i(0, cov_index) = H_low(0, 3);
                H_i(0, cov_index + 1) = H_low(0, 4);
                H_i(1, cov_index) = H_low(1, 3);
                H_i(1, cov_index + 1) = H_low(1, 4);
                // Kalman Gain berechnen
                Eigen::MatrixXd K = Cov * H_i.transpose() * (H_i * Cov * H_i.transpose() + Q).inverse();
                // calculate actual range bearing measurement of LM throug ICP matching of laser scan and stored LM point clouds
                Eigen::Vector2d actualMeasurement = searchLMmatches(pcl_laserscan, convergedLMs, i);

                if (actualMeasurement(0) != -1)
                {
                    numberOfDetectedLandmarks++;
                    Eigen::MatrixXd d = (actualMeasurement - predictedMeasurment);
                    d(1) = constrain_angle(d(1));
                    state_vector = state_vector + K * d;
                    Cov = (Eigen::MatrixXd::Identity(Cov.rows(), Cov.rows()) - K * H_i) * Cov; // Cov updaten
                }
                else
                {
                    // skip the update step
                }
            }
            *storedLMs += stored_landmarks[i].pointcloud;
        }

        // Publish predicted and actual measurement
        predictedMeasPublisher.publish(predictedMeas);
        measuredLMposePublisher.publish(measuredLMpose);

        sensor_msgs::PointCloud2 ros_stored_landmarks;
        sensor_msgs::PointCloud2 ros_converged_landmarks;
        pcl::toROSMsg(*storedLMs.get(), ros_stored_landmarks);
        pcl::toROSMsg(*convergedLMs.get(), ros_converged_landmarks);
        ros_stored_landmarks.header.stamp = ros::Time::now();
        ros_stored_landmarks.header.frame_id = "odom";
        ros_converged_landmarks.header.stamp = ros::Time::now();
        ros_converged_landmarks.header.frame_id = "velodyne";
        storedLandmarksPublisher.publish(ros_stored_landmarks);
        convergedLandmarksPublisher.publish(ros_converged_landmarks);
    }

    Eigen::Vector2d searchLMmatches(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan, pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLMs, int i)
    {
        int cov_index = 3 + 2 * i;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_landmark_target(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Vector2d actualMeasurement;

        *pcl_landmark_target = stored_landmarks[i].pointcloud;
        geometry_msgs::TransformStamped toVelodynetransform;
        // ToDo: statt punktwolke zu verschieben Initial guess bei icp.align() angeben
        try
        {
            toVelodynetransform = tfBuffer.lookupTransform("velodyne", "odom", ros::Time(0), ros::Duration(1.5));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            actualMeasurement(0) = -1;
            return actualMeasurement;
        }

        tf::Transform transform;
        tf::transformMsgToTF(toVelodynetransform.transform, transform);
        geometry_msgs::TransformStamped toOdomtransform;
        tf::transformTFToMsg(transform.inverse(), toOdomtransform.transform);

        pcl::transformPointCloud(*pcl_landmark_target, *pcl_landmark_target, tf2::transformToEigen(toVelodynetransform).matrix());
        // The Iterative Closest Point algorithm
        // ToDo: Fine tunen
        // ToDo: Kann die PCL den Coherent Point Drift (CPD) Algorithmus???? (https://github.com/gadomski/cpd)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(pcl_landmark_target);
        icp.setInputTarget(pcl_laserscan);
        icp.align(*pcl_landmark_target);

        double fitnessScore = icp.getFitnessScore();

        if (fitnessScore < fitnessScoreThreshhold)
        {
            *convergedLMs += *pcl_landmark_target;
            meanFitnessScore += fitnessScore;

            geometry_msgs::Point landmarkPose(stored_landmarks[i].pose);
            tf2::doTransform(landmarkPose, landmarkPose, toVelodynetransform);
            publishMeasuredLMpose(landmarkPose, i * 2);

            double x_mean = 0;
            double y_mean = 0;
            for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_landmark_target->begin(); it != pcl_landmark_target->end(); it++)
            {
                x_mean += it->x;
                y_mean += it->y;
            }
            x_mean /= pcl_landmark_target->size();
            y_mean /= pcl_landmark_target->size();
            landmarkPose.x = x_mean;
            landmarkPose.y = y_mean;

            publishMeasuredLMpose(landmarkPose, i * 2 + 1);
            tf2::doTransform(landmarkPose, landmarkPose, toOdomtransform);

            Eigen::Vector2d delta_actual;
            delta_actual(0) = landmarkPose.x - state_vector(0);
            delta_actual(1) = landmarkPose.y - state_vector(1);

            double q = delta_actual.transpose() * delta_actual;
            actualMeasurement(0) = sqrt(q);
            actualMeasurement(1) = constrain_angle(atan2(delta_actual(1), delta_actual(0)) - state_vector(2));
        }
        else
        {
            actualMeasurement(0) = -1;
        }
        return actualMeasurement;
    }

    void initStateVectorWithLMs(json stored_landmarks_json)
    {
        int cov_index = 0;
        int i = 0;
        // for-Schleife durch alle gespeicherten LM's
        for (json::iterator it = stored_landmarks_json["landmarks"].begin(); it != stored_landmarks_json["landmarks"].end(); ++it)
        {
            cov_index = 3 + 2 * i;

            state_vector(cov_index) = (double)it.value()["pose"].at(0).get<float>();
            state_vector(cov_index + 1) = (double)it.value()["pose"].at(1).get<float>();

            stored_landmarks[i].pose.x = (double)it.value()["pose"].at(0).get<float>();
            stored_landmarks[i].pose.y = (double)it.value()["pose"].at(1).get<float>();
            stored_landmarks[i].pose.z = 0;

            stored_landmarks[i].pointcloud.points.resize(it.value()["points"].size());
            int j = 0;
            for (json::iterator point_it = it.value()["points"].begin(); point_it != it.value()["points"].end(); ++point_it)
            {
                stored_landmarks[i].pointcloud.points[j].x = point_it.value().at(0).get<float>();
                stored_landmarks[i].pointcloud.points[j].y = point_it.value().at(1).get<float>();
                stored_landmarks[i].pointcloud.points[j].z = point_it.value().at(2).get<float>();
                j++;
            }
            i++;
        }
        numberOfStoredLandmarks = i;
    }

    bool newLMcallback(master_thesis_kremmel::Landmark::Request &req, master_thesis_kremmel::Landmark::Response &res)
    {
        int cov_index = 3 + 2 * numberOfStoredLandmarks;
        state_vector(cov_index) = (double)req.x;
        state_vector(cov_index + 1) = (double)req.y;

        stored_landmarks[numberOfStoredLandmarks].pose.x = (double)req.x;
        stored_landmarks[numberOfStoredLandmarks].pose.y = (double)req.y;
        stored_landmarks[numberOfStoredLandmarks].pose.z = 0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(req.points, stored_landmarks[numberOfStoredLandmarks].pointcloud);

        numberOfStoredLandmarks += 1;
        res.success = true;
        return true;
    }

    bool moveCurrentStateCallback(master_thesis_kremmel::MoveRobot::Request &req, master_thesis_kremmel::MoveRobot::Response &res)
    {
        state_vector(0) += (double)req.x;
        state_vector(1) += (double)req.y;
        state_vector(2) += (double)(req.t * M_PI) / 180;

        return true;
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

    tf2::Quaternion getQuat(double theta)
    {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);
        return quat;
    }

    void publishPredictedMeas(int i)
    {
        int cov_index = 3 + 2 * i;
        predictedMeas.header.frame_id = "odom";
        predictedMeas.header.stamp = ros::Time::now();
        predictedMeas.ns = "predicted_measurement";
        predictedMeas.action = visualization_msgs::Marker::ADD;
        predictedMeas.pose.orientation.w = 1.0;
        predictedMeas.id = 1;
        predictedMeas.type = visualization_msgs::Marker::LINE_LIST;
        predictedMeas.scale.x = 0.03;
        predictedMeas.color.r = 1.0;
        predictedMeas.color.b = 0.6;
        predictedMeas.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = state_vector(0);
        p.y = state_vector(1);
        p.z = 0;
        predictedMeas.points.push_back(p);
        p.x = state_vector(cov_index);
        p.y = state_vector(cov_index + 1);
        predictedMeas.points.push_back(p);
    }

    void publishMeasuredLMpose(geometry_msgs::Point landmarkPose, int i)
    {
        int cov_index = 3 + 2 * i;
        measuredLMpose.header.frame_id = "velodyne";
        measuredLMpose.header.stamp = ros::Time::now();
        measuredLMpose.ns = "measured_landmark_pose";
        measuredLMpose.action = visualization_msgs::Marker::ADD;
        measuredLMpose.pose.orientation.w = 1.0;
        measuredLMpose.id = i;
        measuredLMpose.type = visualization_msgs::Marker::LINE_LIST;
        measuredLMpose.scale.x = 0.03;
        measuredLMpose.color.r = 0.5;
        measuredLMpose.color.g = 1.0;
        measuredLMpose.color.a = 1.0;

        measuredLMpose.points.push_back(landmarkPose);
        landmarkPose.z = 1;
        measuredLMpose.points.push_back(landmarkPose);
    }

    void publishEllipses()
    {
        double poseMajor, poseMinor, poseTheta;
        poseEllipse(poseMajor, poseMinor, poseTheta);
        publishBelieve(state_vector(0), state_vector(1), poseTheta, poseMinor, poseMajor, "pose estimate");

        double x, y, lmMajor, lmMinor, lmTheta;
        for (int i = 0; i < numberOfStoredLandmarks; i++)
        {
            landmarkEllipse(i, x, y, lmMinor, lmMajor, lmTheta);
            if (lmMinor < 20 && lmMajor < 20)
            {
                publishBelieve(x, y, lmTheta, lmMinor, lmMajor, "lm estimates", i);
            }
        }
    }

    void poseEllipse(double &major, double &minor, double &theta)
    {
        ellipse(Cov.block(0, 0, 2, 2), major, minor, theta);
    }

    void landmarkEllipse(int id, double &x, double &y, double &minor, double &major, double &theta)
    {
        int idx = 3 + id * 2;
        x = state_vector(idx);
        y = state_vector(idx + 1);
        ellipse(Cov.block(idx, idx, 2, 2), major, minor, theta);
    }

    void ellipse(Eigen::MatrixXd X, double &major, double &minor, double &theta)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> a(X);
        double e0 = sqrt(a.eigenvalues()(0));
        double e1 = sqrt(a.eigenvalues()(1));
        if (e0 > e1)
        {
            theta = atan2(a.eigenvectors()(1, 0), a.eigenvectors()(0, 0));
            major = e0;
            minor = e1;
        }
        else
        {
            theta = atan2(a.eigenvectors()(1, 1), a.eigenvectors()(0, 1));
            major = e1;
            minor = e0;
        }
    }

    void publishBelieve(double x, double y, double theta, double minor, double major, std::string ellipseName, int id = 0)
    {
        //  state_vektor und Cov in Odometry Message verpacken und publishen
        tf2::Quaternion quat = getQuat(theta);
        geometry_msgs::Pose ellipsePose;
        ellipsePose.position.x = x;
        ellipsePose.position.y = y;
        ellipsePose.position.z = 0;
        ellipsePose.orientation = tf2::toMsg(quat);
        // rviz Marker generieren
        visualization_msgs::Marker covEllipseMsg;
        covEllipseMsg.header.frame_id = "odom";
        covEllipseMsg.header.stamp = ros::Time::now();
        // Marker mit selbem Namespace und id überschreiben sich selber (das wollen wir)
        covEllipseMsg.ns = ellipseName;
        covEllipseMsg.id = id;
        covEllipseMsg.type = visualization_msgs::Marker::SPHERE;
        covEllipseMsg.action = visualization_msgs::Marker::ADD;
        covEllipseMsg.pose = ellipsePose;
        covEllipseMsg.scale.x = major;
        covEllipseMsg.scale.y = minor;
        covEllipseMsg.scale.z = 0.01;
        covEllipseMsg.color.r = 1.0f;
        covEllipseMsg.color.g = 0.0f;
        covEllipseMsg.color.b = 1.0f;
        covEllipseMsg.color.a = 0.5;
        covEllipseMsg.lifetime = ros::Duration();
        covEllipseMsgPublisher.publish(covEllipseMsg);

        if (ellipseName == "pose estimate")
        {
            quat = getQuat(state_vector(2));
            ellipsePose.orientation = tf2::toMsg(quat);
            nav_msgs::Odometry currentStateMsg;
            currentStateMsg.pose.pose = ellipsePose;
            currentStateMsg.child_frame_id = "base_footprint";
            currentStateMsg.header.frame_id = "odom";
            currentStateMsg.header.stamp = ros::Time::now();
            currentStatePublisher.publish(currentStateMsg);
        }
    }

    void sendNewBaseLinkTransform()
    {
        tf2::Quaternion quat = getQuat(state_vector(2));
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = state_vector(0);
        transformStamped.transform.translation.y = state_vector(1);
        transformStamped.transform.translation.z = 0;
        transformStamped.transform.rotation = tf2::toMsg(quat);
        br.sendTransform(transformStamped);
    }

private:
    ros::NodeHandle n;
    ros::Publisher covEllipseMsgPublisher;
    ros::Publisher wheelOdometryPublisher;
    ros::Publisher velocityPublisher;
    ros::Publisher numOfDetectedLmPublisher;
    ros::Publisher currentStatePublisher;
    ros::Publisher storedLandmarksPublisher;
    ros::Publisher convergedLandmarksPublisher;
    ros::Publisher predictedMeasPublisher;
    ros::Publisher measuredLMposePublisher;

    message_filters::Subscriber<sensor_msgs::PointCloud2> laserScanSubscriber;
    message_filters::Subscriber<sensor_msgs::JointState> jointStateSubcriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::PointCloud2> SyncRule;
    typedef message_filters::Synchronizer<SyncRule> Sync;
    boost::shared_ptr<Sync> sync;

    ros::ServiceServer newLMservice;
    ros::ServiceServer moveCurrentStateService;

    double last_timestamp; // Zeitspanne die zwischen den Funktionsaufrufen vergeht
    int DIMsize;
    int EPS; // Threshhold under Omega is treated as zero.
    double robotBase;
    double robotWheelRadius;
    double fitnessScoreThreshhold;

    Eigen::Vector3d state_vector_wheel_odom; // Zustandsvektor ohne Correction...also nur Wheel Odomentry
    Eigen::VectorXd state_vector;            // Zustands Vektor der Pose und der Landmarken
    Eigen::VectorXd state_vector_last;       // Zustands Vektor der Pose und der Landmarken bei letztem Aufruf der Predict Methode
    Eigen::MatrixXd Cov;                     // Covarianzmatrix des Zustandsvekotors (Roboterpose und Landmarken)
    int numberOfStoredLandmarks;
    int numberOfDetectedLandmarks;

    double alphaRightLast;
    double alphaLeftLast;

    struct Landmark
    {
        geometry_msgs::Point pose; // mean of all landmark points
        pcl::PointCloud<pcl::PointXYZ> pointcloud;
    };

    Landmark *stored_landmarks;

    Eigen::Matrix2d Q;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *listener;

    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    visualization_msgs::Marker predictedMeas, actualMeas, measuredLMpose;

    double meanFitnessScore;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    EKFSlam Node(n);

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}