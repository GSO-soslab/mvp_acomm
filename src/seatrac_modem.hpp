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

//ros includes for syncronizing voltage and current subscribers
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

//ros msg includes for pose
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geographic_msgs/GeoPoint.h"
//ros msg includes for health
#include "mvp_msgs/Float64Stamped.h"
//ros service includes for controller info
#include "std_srvs/Empty.h"
//ros msg includes for direct control
#include "mvp_msgs/ControlProcess.h"
//ros service for change state and get state
#include "mvp_msgs/ChangeState.h"
//ros msg include for waypoints
#include "geometry_msgs/PolygonStamped.h"
//ros msg include for depth setpoint
#include "std_msgs/Float64.h"

//robot localization include to transform local x,y and lat and long
#include "robot_localization/ToLL.h"
#include "robot_localization/FromLL.h"

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



class SeaTracModem{

private:
    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;

    ros::Publisher health_pub;
    ros::Publisher pose_pub;
    ros::Publisher direct_control_pub;
    ros::Publisher update_waypoint_pub;
    ros::Publisher append_waypoint_pub;

    ros::Subscriber local_odom_sub;

    ros::ServiceClient controller_enable_client;
    ros::ServiceClient controller_disable_client;
    ros::ServiceClient set_state_client;
    ros::ServiceClient get_state_client;

    bool console_debug;
    int beacon_id;
    uint32_t our_id;
    uint32_t dest_id;
    uint32_t slot_time;
    geometry_msgs::PolygonStamped waypoint_array;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();
    goby::acomms::QueueManager q_manager;
    goby::acomms::SeatracDriver st_driver;
    goby::acomms::MACManager mac;

    goby::acomms::protobuf::QueueManagerConfig q_manager_cfg;

    void setup_goby();
    void setup_queue();
    void received_data(const google::protobuf::Message& data_msg);
    void received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message, const google::protobuf::Message& original_message);
    void f_local_odom_callback(const nav_msgs::OdometryPtr &msg);
    void f_power_callback(const mvp_msgs::Float64Stamped &voltage, const mvp_msgs::Float64Stamped &current);
    ros::Timer timer;
    
    PoseResponse pose_out;
    HealthResponse health_out;

public:

    SeaTracModem();
    ~SeaTracModem();

};

