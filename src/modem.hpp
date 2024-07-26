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
#include <mvp_msgs/Power.h>
#include <sensor_msgs/NavSatFix.h>
#include <alpha_comms/AcommsRx.h>
#include <alpha_comms/AcommsTx.h>

#include "robot_localization/ToLL.h"

#include <geographic_msgs/GeoPoseStamped.h>

//goby includes
#include <goby/acomms/connect.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/buffer/dynamic_buffer.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>


#include "goby/util/debug_logger/flex_ostream.h"         // for FlexOs...
#include "goby/util/debug_logger/flex_ostreambuf.h"      // for DEBUG1

//protobuf msg includes
#include "proto/goby_msgs.pb.h"

//driver includes
#include "seatrac_driver.h"
#include "evologics_driver.h"

#include "common.h"

class Modem{

public:

    Modem();
    ~Modem();

    

private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;

    ros::Subscriber modem_tx_;
    ros::Publisher modem_rx_;

    ros::Publisher track_pub_;
    ros::Subscriber gps_sub_;

    ros::ServiceClient toll_;



    struct Interface
    {
        std::string if_type;
        std::string tcp_address;
        int tcp_port;
        std::string device;
        int baudrate;
    };

    struct DynamicBuffer
    {
        std::vector<std::string> messages;
    };

    struct MessageConfig
    {
        bool ack;
        int blackout_time;
        int max_queue;
        bool newest_first;
        int ttl;
        int value_base;
    };

    std::map<std::string, MessageConfig> dynamic_buffer_config_;

    struct Config
    {
        std::string type;
        std::string driver;
        int max_frame_bytes;
        int mac_slot_time;

        DynamicBuffer dynamic_buffer;

        Interface interface;
        int source_level;
        int source_control;
        int gain_level;
        int carrier_waveform_id;
        int local_address;
        int remote_address;
        int highest_address;
        int cluster_size;
        int packet_time;
        int retry_count;
        int retry_timeout;
        int keep_online_count;
        int idle_timeout;
        int channel_protocol_id;
        int sound_speed;
    };

    goby::acomms::MACManager mac;

    void loop();
    void loadGoby();
    void loadBuffer();
    void configModem();
    void parseGobyParams();
    void parseEvologicsParams();
    void parseSeatracParams();
    void dataRequest(goby::acomms::protobuf::ModemTransmission *msg);
    void addToBuffer(const alpha_comms::AcommsTxConstPtr& msg);
    void receivedData(const goby::acomms::protobuf::ModemTransmission &data_msg);
    void evologicsPositioningData(UsbllongMsg msg);
    void seatracPositioningData(ACOFIX_T msg);
    void onGps(const sensor_msgs::NavSatFixConstPtr fix);

    goby::acomms::EvologicsDriver evo_driver_;
    goby::acomms::SeatracDriver seatrac_driver_;

    goby::acomms::DynamicBuffer<std::string> buffer_;

    ros::Timer timer_;

    Config config_;

    sensor_msgs::NavSatFix fix_;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();
    



};

