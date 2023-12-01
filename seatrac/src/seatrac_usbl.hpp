/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller
    Email: jason_miller@uri.edu
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/
#pragma once

#include <thread>
#include <stdlib.h>
#include "ros/ros.h"
#include "unistd.h"
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <mvp_msgs/VehicleStatus.h>
#include <mvp_msgs/CommsPose.h>
#include <mvp_msgs/CommsHealth.h>
#include <mvp_msgs/CommsRelativePose.h>
#include <mvp_msgs/CommsControllerInfo.h>
#include <mvp_msgs/CommsDirectControl.h>
#include <mvp_msgs/CommsStateInfo.h>
#include <mvp_msgs/CommsSingleWaypoint.h>
#include <mvp_msgs/CommsMultiWaypointGPS.h>
#include <mvp_msgs/CommsMultiWaypointXYZ.h>
#include <mvp_msgs/CommsExecuteWaypoint.h>

//goby includes
#include <goby/acomms/connect.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>

//protobuf msg includes
#include "proto/goby_msgs.pb.h"

//driver includes
#include "seatrac_driver.h"

class SeaTracUSBL{

private:
    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;
    
    ros::Publisher odom_pub;
    ros::Publisher health_pub;
    
    ros::ServiceServer pose_server;
    ros::ServiceServer health_server;
    ros::ServiceServer relative_pose_server;
    ros::ServiceServer controller_info_server;
    ros::ServiceServer direct_control_server;
    ros::ServiceServer state_info_server;
    ros::ServiceServer single_waypoint_server;
    ros::ServiceServer multi_waypoint_gps_server;
    ros::ServiceServer multi_waypoint_xyz_server;
    ros::ServiceServer execute_waypoints_server;

    bool console_debug;
    int beacon_id;
    uint32_t our_id;
    uint32_t dest_id;
    uint32_t slot_time;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();
    goby::acomms::QueueManager q_manager;
    goby::acomms::SeatracDriver st_driver;
    goby::acomms::MACManager mac;

    goby::acomms::protobuf::QueueManagerConfig q_manager_cfg;

    void setup_goby();
    void setup_queue();
    void received_data(const google::protobuf::Message& message_in);
    void received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message, const google::protobuf::Message& original_message);
    bool f_cb_srv_request_pose(mvp_msgs::CommsPose::Request &request, mvp_msgs::CommsPose::Response &response);
    bool f_cb_srv_request_health(mvp_msgs::CommsHealth::Request &request, mvp_msgs::CommsHealth::Response &response);
    bool f_cb_srv_request_relative_pose(mvp_msgs::CommsRelativePose::Request &request, mvp_msgs::CommsRelativePose::Response &response);
    bool f_cb_srv_controller_info(mvp_msgs::CommsControllerInfo::Request &request, mvp_msgs::CommsControllerInfo::Response &response);
    bool f_cb_srv_direct_control(mvp_msgs::CommsDirectControl::Request &request, mvp_msgs::CommsDirectControl::Response &response);
    bool f_cb_srv_state_info(mvp_msgs::CommsStateInfo::Request &request, mvp_msgs::CommsStateInfo::Response &response);
    bool f_cb_srv_single_waypoint(mvp_msgs::CommsSingleWaypoint::Request &request, mvp_msgs::CommsSingleWaypoint::Response &response);
    bool f_cb_srv_multi_waypoint_gps(mvp_msgs::CommsMultiWaypointGPS::Request &request, mvp_msgs::CommsMultiWaypointGPS::Response &response);
    bool f_cb_srv_multi_waypoint_xyz(mvp_msgs::CommsMultiWaypointXYZ::Request &request, mvp_msgs::CommsMultiWaypointXYZ::Response &response);
    bool f_cb_srv_execute_waypoints(mvp_msgs::CommsExecuteWaypoint::Request &request, mvp_msgs::CommsExecuteWaypoint::Response &response);







    ros::Timer timer;
    

public:

    SeaTracUSBL();
    ~SeaTracUSBL();

};

