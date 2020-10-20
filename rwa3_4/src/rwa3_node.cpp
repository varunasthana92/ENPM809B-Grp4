// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <tf2_ros/transform_listener.h>
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>

// To store quality sensor model detected
nist_gear::LogicalCameraImage quality1_model;

void transform_pose(geometry_msgs::Pose input_pose, 
                    std::string to_frame, 
                    std::string from_frame) {
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::Buffer tfBuffer;
    transformStamped = tfBuffer.lookupTransform("world", "quality_control_sensor_1_frame", ros::Time(0));
}

void orderCallback(const nist_gear::Order& ordermsg) {
    Order order_recieved;
    Product product_recieved;
    Shipment shipment_recieved;
    order_recieved.order_id = ordermsg.order_id;
    for(const auto &ship: ordermsg.shipments) {
        shipment_recieved.shipment_type = ship.shipment_type;
        shipment_recieved.agv_id = ship.agv_id;
        for(const auto &prod: ship.products) {
            product_recieved.type = prod.type;
            product_recieved.pose = prod.pose;
            shipment_recieved.products.emplace_back(product_recieved);
        }
        order_recieved.shipments.push_back(shipment_recieved);
    }
    ROS_INFO_STREAM("I heard: " << order_recieved.order_id);
    for(auto s: order_recieved.shipments) {
        ROS_INFO_STREAM("Order type: " << s.shipment_type);
    }
}

void qualityCallback(const nist_gear::LogicalCameraImage& msg) {
    quality1_model = msg;
}

bool checkPartPoseValidity (part part_in_tray, 
                            nist_gear::LogicalCameraImage quality1_model) {
    ROS_INFO_STREAM("Order part location to be placed to: " 
                    << part_in_tray.pose.position.x 
                    << part_in_tray.pose.position.y
                    << part_in_tray.pose.position.z
                    << part_in_tray.pose.orientation.x
                    << part_in_tray.pose.orientation.y
                    << part_in_tray.pose.orientation.z
                    << part_in_tray.pose.orientation.w);
    ROS_INFO_STREAM("Order part pose detected from quality sensor: " 
                    << quality1_model.pose.position.x 
                    << quality1_model.pose.position.y
                    << quality1_model.pose.position.z
                    << quality1_model.pose.orientation.x
                    << quality1_model.pose.orientation.y
                    << quality1_model.pose.orientation.z
                    << quality1_model.pose.orientation.w);
    // Transform pose detected from quality sensor to world frame
    
}
int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(50);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber order_sub = node.subscribe("/ariac/orders", 1000, orderCallback);
    ros::Subscriber quality1_sub = node.subscribe("/ariac/quality_control_sensor_1", 1000, qualityCallback);
    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    gantry.goToPresetLocation(gantry.bin3_);


    //--You should receive the following information from a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
    part part_in_tray;
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.12;
    part_in_tray.pose.position.y = -0.2;
    part_in_tray.pose.position.z = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.0;
    part_in_tray.pose.orientation.w = 1.0;

    //--Go pick the part
    gantry.pickPart(my_part);
    //--Go place the part
    gantry.placePart(part_in_tray, "agv2");

    bool placed_status = checkPartPoseValidity(part_in_tray, quality1_model);

    ROS_INFO_STREAM("Part placed correctly: " << placed_status);

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}