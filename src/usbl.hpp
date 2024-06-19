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


//goby includes
#include <goby/acomms/connect.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/buffer/dynamic_buffer.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>

#include <goby/middleware/group.h>
#include <goby/middleware/application/thread.h>
#include <goby/middleware/transport/interprocess.h>
#include <goby/middleware/transport/interthread.h>
#include <goby/middleware/protobuf/intervehicle.pb.h>
#include <goby/middleware/protobuf/serializer_transporter.pb.h>
#include <goby/middleware/protobuf/serializer_transporter.pb.h>
#include <goby/middleware/marshalling/dccl.h>
#include "goby/middleware/protobuf/intervehicle.pb.h"

#include "goby/util/debug_logger/flex_ostream.h"         // for FlexOs...
#include "goby/util/debug_logger/flex_ostreambuf.h"      // for DEBUG1

//protobuf msg includes
#include "proto/goby_msgs.pb.h"

//driver includes
#include "seatrac_driver.h"
#include "evologics_driver.h"

#include "common.h"

class USBL{

public:

    USBL();
    ~USBL();

    

private:
    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;

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

    goby::acomms::EvologicsDriver evo_driver;
    goby::acomms::MACManager mac;

    void loop();
    void load_goby();
    void load_buffer();
    void configure_usbl();
    void parse_goby_params();
    void parse_evologics_params();
    void data_request(goby::acomms::protobuf::ModemTransmission* msg);
    void received_data(const goby::acomms::protobuf::ModemTransmission &data_msg);

    goby::acomms::DynamicBuffer<std::string> buffer_;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();

    ros::Timer timer;

    Config config_;
    


};

