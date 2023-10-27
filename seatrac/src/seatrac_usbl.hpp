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
#include <std_msgs/Header.h>
#include <mvp_msgs/ModemHealth.h>
#include <mvp_msgs/ModemPose.h>
#include <std_msgs/Header.h>
#include "ros/ros.h"
#include "unistd.h"
#include <iostream>

#include <goby/acomms/connect.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/dccl.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>
#include "proto/goby_msgs.pb.h"
#include "seatrac_driver.h"

#include "io.h"

#include <vector>

class CSeatrac;

class SeaTracUSBL{

private:
    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;
    
    ros::Publisher health_publisher;
    ros::Publisher pose_publisher;

    bool console_debug;
    int beacon_id;

    std::string buffer;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();
    goby::acomms::QueueManager q_manager;
    goby::acomms::SeatracDriver st_driver;
    goby::acomms::MACManager mac;

    goby::acomms::protobuf::QueueManagerConfig q_manager_cfg;

    void setup_goby();
    void setup_msgs();
    void received_data(const google::protobuf::Message& message_in);
    void received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message, const google::protobuf::Message& original_message);
    ros::Timer timer;
    

public:

    SeaTracUSBL();
    ~SeaTracUSBL();

};

