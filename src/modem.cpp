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

#include "modem.hpp"

using goby::util::as;
using goby::glog;

/**
 * @brief Construct a Modem object
 * 
 */
Modem::Modem()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    parse_goby_params();

    if(config_.driver == "evologics"){ parse_evologics_params(); };

    load_goby();

    configure_modem();

    m_modem_tx = m_nh->subscribe("modem/tx", 10, &Modem::transmit_buffer, this);
    m_modem_rx = m_nh->advertise<alpha_acomms::AcommsRx>("modem/rx", 10);

    loop();

}

/**
 * @brief Destroy the Modem:: Modem object
 * 
 */
Modem::~Modem()
{

}

void Modem::loop()
{
    ros::Rate rate(10);


    //loop at 10Hz 
    while(ros::ok())
    {
        evo_driver.do_work();
        mac.do_work();

        ros::spinOnce();

        rate.sleep();
    }
}

void Modem::parse_goby_params()
{
    m_pnh->param<std::string>("goby/driver", config_.driver, "evologics");
    m_pnh->param<int>("goby/max_frame_bytes", config_.max_frame_bytes, 100);
    m_pnh->param<int>("goby/mac_slot_time", config_.mac_slot_time, 10);

    m_pnh->param<std::vector<std::string>>("goby/dynamic_buffer/messages", config_.dynamic_buffer.messages, {""});

    for( std::string message : config_.dynamic_buffer.messages)
    {
        m_pnh->param<bool>("goby/dynamic_buffer/" + message + "/ack", dynamic_buffer_config_[message].ack, false);
        m_pnh->param<int>("goby/dynamic_buffer/" + message + "/blackout_time", dynamic_buffer_config_[message].blackout_time, 0);
        m_pnh->param<int>("goby/dynamic_buffer/" + message + "/max_queue", dynamic_buffer_config_[message].max_queue, 0);
        m_pnh->param<bool>("goby/dynamic_buffer/" + message + "/newest_first", dynamic_buffer_config_[message].newest_first, true);
        m_pnh->param<int>("goby/dynamic_buffer/" + message + "/ttl", dynamic_buffer_config_[message].ttl, 1800);
        m_pnh->param<int>("goby/dynamic_buffer/" + message + "/value_base", dynamic_buffer_config_[message].value_base, 1);
    }
}

void Modem::parse_evologics_params()
{
    m_pnh->param<std::string>("modem_configuration/interface/connection_type", config_.interface.if_type, "tcp");
    m_pnh->param<std::string>("modem_configuration/interface/tcp_address", config_.interface.tcp_address, "192.168.2.109");
    m_pnh->param<int>("modem_configuration/interface/tcp_port", config_.interface.tcp_port, 9200);
    m_pnh->param<std::string>("modem_configuration/interface/device", config_.interface.device, "/dev/ttyUSB0");
    m_pnh->param<int>("modem_configuration/interface/baudrate", config_.interface.baudrate, 115200);
    m_pnh->param<int>("modem_configuration/source_level", config_.source_level, 0);
    m_pnh->param<int>("modem_configuration/source_control", config_.source_control,1);
    m_pnh->param<int>("modem_configuration/gain_level", config_.gain_level, 0);
    m_pnh->param<int>("modem_configuration/carrier_waveform_id", config_.carrier_waveform_id, 0);
    m_pnh->param<int>("modem_configuration/local_address", config_.local_address, 1);
    m_pnh->param<int>("modem_configuration/remote_address", config_.remote_address, 2);
    m_pnh->param<int>("modem_configuration/highest_address", config_.highest_address, 2);
    m_pnh->param<int>("modem_configuration/cluster_size", config_.cluster_size, 10);
    m_pnh->param<int>("modem_configuration/packet_time", config_.packet_time, 750);
    m_pnh->param<int>("modem_configuration/retry_count", config_.retry_count, 3);
    m_pnh->param<int>("modem_configuration/retry_timeout", config_.retry_timeout, 4000);
    m_pnh->param<int>("modem_configuration/keep_online_count", config_.keep_online_count, 0);
    m_pnh->param<int>("modem_configuration/idle_timeout", config_.idle_timeout, 120);
    m_pnh->param<int>("modem_configuration/channel_protocol_id", config_.channel_protocol_id, 0);
    m_pnh->param<int>("modem_configuration/sound_speed", config_.sound_speed, 1500);
}



/**
 * @brief the goby dccl, mac, queue, and driver are configured and initialized
 * 
 */
void Modem::load_goby()
{
    goby::acomms::bind(mac, evo_driver);

    // connect the receive signal from the driver to the modem slot
    goby::acomms::connect(&evo_driver.signal_receive, this, &Modem::received_data);

    // connect the outgoing data request signal from the driver to the modem slot
    goby::acomms::connect(&evo_driver.signal_data_request, this, &Modem::data_request);

    dccl_->validate<PoseCommand>();
    dccl_->validate<PowerCommand>();

    //Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;

    driver_cfg.set_modem_id(config_.local_address);

    if(config_.interface.if_type == "tcp")
    {
        driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_TCP_AS_CLIENT);
        driver_cfg.set_tcp_server(config_.interface.tcp_address);
        driver_cfg.set_tcp_port(config_.interface.tcp_port);
    }
    else if (config_.interface.if_type == "serial")
    {
        driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);
        driver_cfg.set_serial_port(config_.interface.device);
        driver_cfg.set_serial_baud(config_.interface.baudrate);
    }

    //Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(config_.local_address);

    // setup our modem slot
    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(config_.local_address);
    my_slot.set_dest(config_.remote_address);
    my_slot.set_max_frame_bytes(config_.max_frame_bytes);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);

    // setup the modem slot
    goby::acomms::protobuf::ModemTransmission buddy_slot;
    buddy_slot.set_src(config_.remote_address);
    buddy_slot.set_dest(config_.local_address);
    buddy_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    buddy_slot.set_slot_seconds(config_.mac_slot_time);

    if (config_.local_address < config_.remote_address)
    {
        mac_cfg.add_slot()->CopyFrom(my_slot);
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
    }
    else
    {
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
        mac_cfg.add_slot()->CopyFrom(my_slot);
    }

    goby::glog.set_name("modem");
    goby::glog.add_stream(goby::util::logger::DEBUG1, &std::clog);

    // startup the mac and evo_driver
    mac.startup(mac_cfg);
    evo_driver.startup(driver_cfg);

    load_buffer();
}

void Modem::load_buffer()
{
    // create a buffer cfg
    goby::acomms::protobuf::DynamicBufferConfig cfg;

    std::map<std::string, MessageConfig>::iterator it = dynamic_buffer_config_.begin();

    while(it != dynamic_buffer_config_.end())
    {

        cfg.set_ack_required(it->second.ack);
        cfg.set_blackout_time(it->second.blackout_time);
        cfg.set_max_queue(it->second.max_queue);
        cfg.set_newest_first(it->second.newest_first);
        cfg.set_ttl(it->second.ttl);
        cfg.set_value_base(it->second.value_base);

        buffer_.create(config_.remote_address, it->first, cfg);

        cfg.Clear();

        ++it;
    }
    // set the 
}

void Modem::configure_modem()
{
    if(config_.driver == "evologics")
    {

        evo_driver.set_source_level(config_.source_level);

        evo_driver.set_source_control(config_.source_control);

        evo_driver.set_gain(config_.gain_level);

        evo_driver.set_carrier_waveform_id(config_.carrier_waveform_id);

        evo_driver.set_local_address(config_.local_address);

        evo_driver.set_remote_address(config_.remote_address);

        evo_driver.set_highest_address(config_.highest_address);

        evo_driver.set_cluster_size(config_.cluster_size);

        evo_driver.set_packet_time(config_.packet_time);

        evo_driver.set_retry_count(config_.retry_count);

        evo_driver.set_retry_timeout(config_.retry_timeout);

        evo_driver.set_keep_online_count(config_.keep_online_count);

        evo_driver.set_idle_timeout(config_.idle_timeout);

        evo_driver.set_channel_protocol_id(config_.channel_protocol_id);

        evo_driver.set_sound_speed(config_.sound_speed);
    }
}

/**
 * @brief slot that the driver calls when it wants to send data
 * 
 * @param msg pointer to the outgoing message the driver is requesting
 */
void Modem::data_request(goby::acomms::protobuf::ModemTransmission* msg)
{
    int dest = msg->dest();
    for (auto frame_number = msg->frame_start(),
              total_frames = msg->max_num_frames() + msg->frame_start();
         frame_number < total_frames; ++frame_number)
    {
        std::string* frame = msg->add_frame();

        while (frame->size() < msg->max_frame_bytes())
        {
            try
            {
                auto buffer_value =
                    buffer_.top(dest, msg->max_frame_bytes() - frame->size());
                dest = buffer_value.modem_id;
                *frame += buffer_value.data.data();

                buffer_.erase(buffer_value);
            }
            catch (goby::acomms::DynamicBufferNoDataException&)
            {
                break;
            }
        }
    }
}

void Modem::transmit_buffer(const alpha_acomms::AcommsTx msg)
{

    if(dynamic_buffer_config_.find(msg.subbuffer_id) != dynamic_buffer_config_.end())
    {
        buffer_.push({config_.remote_address, msg.subbuffer_id , goby::time::SteadyClock::now(), msg.data});    
    }
    else
    {
        ROS_INFO("Subbuffer ID: %s has not been added to the configuratiron file goby.yaml\n", msg.subbuffer_id.data());
    }

}

/**
 * @brief the slot that is called back from the driver when a new message is received.
 * 
 * @param data_msg the incoming message
 */
void Modem::received_data(const goby::acomms::protobuf::ModemTransmission& data_msg)
{
    alpha_acomms::AcommsRx msg;

    msg.data = data_msg.frame()[0];

    m_modem_rx.publish(msg);
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "modem");

    Modem d;

    ros::spin();

    return 0;
}
