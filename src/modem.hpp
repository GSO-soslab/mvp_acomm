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

//ros msg includes for pose
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geographic_msgs/GeoPoint.h"
//ros msg includes for health
#include "mvp_msgs/Power.h"
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
#include <goby/acomms/buffer/dynamic_buffer.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>

//protobuf msg includes
#include "proto/goby_msgs.pb.h"

//driver includes
#include "seatrac_driver.h"
#include "evologics_driver.h"



class Modem{

private:
    ros::NodeHandle m_nh;
    ros::NodeHandlePtr m_pnh;
    
    int our_id_;
    int dest_id_;
    int slot_time_;

    void loop();
    void setup_goby();
    void data_request(goby::acomms::protobuf::ModemTransmission* msg);
    void received_data(const google::protobuf::Message& message_in);

    ros::Timer timer;
    


public:
    
    goby::acomms::DynamicBuffer<std::string> buffer_;

    goby::acomms::EvologicsDriver evo_driver;
    goby::acomms::MACManager mac;


    Modem();
    ~Modem();

};

