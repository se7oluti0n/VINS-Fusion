/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <rclcpp/rclcpp.hpp>
#include "globalOpt.h"
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <fstream>
#include <queue>
#include <mutex>


// ros::Publisher pub_global_odometry, pub_global_path, pub_car;


class GlobalOpNode
{

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_global_odometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_car;

    rclcpp::SubscriptionBase::SharedPtr sub_GPS;
    rclcpp::SubscriptionBase::SharedPtr sub_vio;

    GlobalOptimization globalEstimator;
    nav_msgs::msg::Path *global_path;
    double last_vio_t = -1;
    std::queue<sensor_msgs::msg::NavSatFix::ConstPtr> gpsQueue;
    std::mutex m_buf;
    rclcpp::Node::SharedPtr node_;
public:

    GlobalOpNode(rclcpp::Node::SharedPtr node): node_(node) {
        global_path = &globalEstimator.global_path;
        
        pub_global_odometry =
            node_->create_publisher<::nav_msgs::msg::Odometry>(
                "~/global_odometry", 10);

        pub_global_path = node_->create_publisher<nav_msgs::msg::Path>(
            "~/global_path", 100);

        pub_car = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/car_model", 100);

        sub_GPS = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                            "/gps", rclcpp::SensorDataQoS(), 
                            std::bind(&GlobalOpNode::GPS_callback, this, std::placeholders::_1));
        sub_vio = node_->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 100, 
                std::bind(&GlobalOpNode::vio_callback, this, std::placeholders::_1));

    }

    GlobalOpNode() = delete;

    void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
    {
        visualization_msgs::msg::MarkerArray markerArray_msg;
        visualization_msgs::msg::Marker car_mesh;
        car_mesh.header.stamp = rclcpp::Time(t);
        car_mesh.header.frame_id = "world";
        car_mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        car_mesh.action = visualization_msgs::msg::Marker::ADD;
        car_mesh.id = 0;

        car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

        Eigen::Matrix3d rot;
        rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
        
        Eigen::Quaterniond Q;
        Q = q_w_car * rot; 
        car_mesh.pose.position.x    = t_w_car.x();
        car_mesh.pose.position.y    = t_w_car.y();
        car_mesh.pose.position.z    = t_w_car.z();
        car_mesh.pose.orientation.w = Q.w();
        car_mesh.pose.orientation.x = Q.x();
        car_mesh.pose.orientation.y = Q.y();
        car_mesh.pose.orientation.z = Q.z();

        car_mesh.color.a = 1.0;
        car_mesh.color.r = 1.0;
        car_mesh.color.g = 0.0;
        car_mesh.color.b = 0.0;

        float major_scale = 2.0;

        car_mesh.scale.x = major_scale;
        car_mesh.scale.y = major_scale;
        car_mesh.scale.z = major_scale;
        markerArray_msg.markers.push_back(car_mesh);
        pub_car->publish(markerArray_msg);
    }

    void GPS_callback(const sensor_msgs::msg::NavSatFix::ConstPtr &GPS_msg)
    {
        //printf("gps_callback! \n");
        m_buf.lock();
        gpsQueue.push(GPS_msg);
        m_buf.unlock();
    }

    void vio_callback(const nav_msgs::msg::Odometry::ConstPtr &pose_msg)
    {
        //printf("vio_callback! \n");
        double t =   rclcpp::Time(pose_msg->header.stamp).seconds();
        last_vio_t = t;
        Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = pose_msg->pose.pose.orientation.w;
        vio_q.x() = pose_msg->pose.pose.orientation.x;
        vio_q.y() = pose_msg->pose.pose.orientation.y;
        vio_q.z() = pose_msg->pose.pose.orientation.z;
        globalEstimator.inputOdom(rclcpp::Time(pose_msg->header.stamp), vio_t, vio_q);


        m_buf.lock();
        while(!gpsQueue.empty())
        {
            sensor_msgs::msg::NavSatFix::ConstPtr GPS_msg = gpsQueue.front();
            double gps_t = rclcpp::Time(GPS_msg->header.stamp).seconds();
            printf("vio t: %f, gps t: %f \n", t, gps_t);
            // 10ms sync tolerance
            if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
            {
                //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
                double latitude = GPS_msg->latitude;
                double longitude = GPS_msg->longitude;
                double altitude = GPS_msg->altitude;
                //int numSats = GPS_msg->status.service;
                double pos_accuracy = GPS_msg->position_covariance[0];
                if(pos_accuracy <= 0)
                    pos_accuracy = 1;
                //printf("receive covariance %lf \n", pos_accuracy);
                //if(GPS_msg->status.status > 8)
                    globalEstimator.inputGPS(rclcpp::Time(pose_msg->header.stamp), latitude, longitude, altitude, pos_accuracy);
                gpsQueue.pop();
                break;
            }
            else if(gps_t < t - 0.01)
                gpsQueue.pop();
            else if(gps_t > t + 0.01)
                break;
        }
        m_buf.unlock();

        Eigen::Vector3d global_t;
        Eigen:: Quaterniond global_q;
        globalEstimator.getGlobalOdom(global_t, global_q);

        nav_msgs::msg::Odometry odometry;
        odometry.header = pose_msg->header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        odometry.pose.pose.position.x = global_t.x();
        odometry.pose.pose.position.y = global_t.y();
        odometry.pose.pose.position.z = global_t.z();
        odometry.pose.pose.orientation.x = global_q.x();
        odometry.pose.pose.orientation.y = global_q.y();
        odometry.pose.pose.orientation.z = global_q.z();
        odometry.pose.pose.orientation.w = global_q.w();
        pub_global_odometry->publish(odometry);
        pub_global_path->publish(*global_path);
        publish_car_model(t, global_t, global_q);


        // write result to file
        #if 0
            std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
            foutC.setf(ios::fixed, ios::floatfield);
            foutC.precision(0);
            foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
            foutC.precision(5);
            foutC << global_t.x() << ","
                    << global_t.y() << ","
                    << global_t.z() << ","
                    << global_q.w() << ","
                    << global_q.x() << ","
                    << global_q.y() << ","
                    << global_q.z() << endl;
            foutC.close();
        #endif
    }

};


int main(int argc, char **argv)
{
    // ros::init(argc, argv, "globalEstimator");
    // ros::NodeHandle n("~");
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr ros2_node = rclcpp::Node::make_shared("globalEstimator");

    GlobalOpNode global_op(ros2_node);

    

    // ros::Subscriber sub_GPS = n.subscribe("/gps", 100, GPS_callback);
    // ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    // pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    // pub_global_odometry = n.advertise<nav_msgs::msg::Odometry>("global_odometry", 100);
    // pub_car = n.advertise<visualization_msgs::msg::MarkerArray>("car_model", 1000);
    // ros::spin();
    auto executor = rclcpp::executors::MultiThreadedExecutor();
    executor.add_node(ros2_node);
    executor.spin();
    return 0;
}
