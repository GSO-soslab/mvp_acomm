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

/**
 * @brief Construct a new Modem object
 *
 */
Modem::Modem()
{
    m_pnh.reset(new ros::NodeHandle("~"));

    // local_odom_sub = m_nh.subscribe("/alpha_rise/odometry/filtered/local", 1, &Modem::f_local_odom_callback, this);

    // power_sub = m_nh.subscribe("/alpha_rise/power_monitor/power", 1, &Modem::f_power_callback, this);

    // controller_enable_client = m_nh.serviceClient<std_srvs::Empty>("controller/enable");
    // controller_disable_client = m_nh.serviceClient<std_srvs::Empty>("controller/disable");
    // direct_control_pub = m_nh.advertise<mvp_msgs::ControlProcess>("continuous_command_topic", 10);
    // set_state_client = m_nh.serviceClient<mvp_msgs::ChangeState>("helm/change_state");
    // get_state_client = m_nh.serviceClient<mvp_msgs::ChangeState>("helm/get_state");
    // append_waypoint_pub = m_nh.advertise<geometry_msgs::PolygonStamped>("helm/path_3d/append_waypoints", 10);
    // update_waypoint_pub = m_nh.advertise<geometry_msgs::PolygonStamped>("helm/path_3d/update_waypoints", 10);


    setup_goby();

    loop();

}

/**
 * @brief Destroy the Sea Trac Modem:: Sea Trac Modem object
 *
 */
Modem::~Modem()
{
}

void Modem::loop()
{
    ros::Rate rate(10);

    int i = 0;

    std::string example_value = "hi usbl";

    goby::acomms::DynamicBuffer<std::string>::Value example_msg;

    buffer_.push({dest_id_, "example_subbuffer", goby::time::SteadyClock::now(), example_value});

    //loop at 10Hz 
    while(ros::ok())
    {
        if(i>200)
        {
            buffer_.push({dest_id_, "example_subbuffer", goby::time::SteadyClock::now(), example_value});
            i=0;
        }

        evo_driver.do_work();
        mac.do_work();

        ros::spinOnce();

        rate.sleep();

        i++;
    }
}

/**
 * @brief the goby dccl, mac, queue, and driver are configured and initialized
 *
 */
void Modem::setup_goby()
{
    goby::acomms::bind(mac, evo_driver);

    // set the source id of this modem
    our_id_ = 2;
    dest_id_ = 1;
    slot_time_ = 5;

    // create a buffer cfg
    goby::acomms::protobuf::DynamicBufferConfig cfg;

    // set the queue configuration (https://goby.software/3.0/md_doc101_acomms-queue.html)
    cfg.set_ack_required(false);
    cfg.set_ttl(3000);
    cfg.set_value_base(10);
    cfg.set_max_queue(5);

    // create the subbuffer in the buffer
    buffer_.create(dest_id_, "example_subbuffer", cfg);

    // connect the receive signal from the driver to the USBL slot
    goby::acomms::connect(&evo_driver.signal_receive, this, &Modem::received_data);

    // connect the outgoing data request signal from the driver to the USBL slot
    goby::acomms::connect(&evo_driver.signal_data_request, this, &Modem::data_request);

    //Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;

    // configure the modem driver as TCP Client 192.168.2.108:9200 per current Evologics USBL configuration
    driver_cfg.set_modem_id(our_id_);
    driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_TCP_AS_CLIENT);
    driver_cfg.set_tcp_server("192.168.2.108");
    driver_cfg.set_tcp_port(9200);


    //Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(our_id_);

    // setup our USBL slot
    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(our_id_);
    my_slot.set_dest(dest_id_);
    my_slot.set_rate(0);
    my_slot.set_max_frame_bytes(500);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);

    // setup the modem slot
    goby::acomms::protobuf::ModemTransmission buddy_slot;
    buddy_slot.set_src(dest_id_);
    buddy_slot.set_dest(our_id_);
    buddy_slot.set_rate(0);
    buddy_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    buddy_slot.set_slot_seconds(slot_time_);

    if (our_id_ < dest_id_)
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
}


/**
 * @brief the slot that is called back from evo_driver when a new message is received. the incoming
 * message is parsed acording to the mvp_messages documentation.
 *
 * @param data_msg the incoming protobuf message
 */
void Modem::received_data(const google::protobuf::Message &data_msg)
{
    std::string msg_type =  data_msg.GetTypeName();
    printf("Received %s: %s\n", msg_type.c_str(), data_msg.ShortDebugString().c_str());
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

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "seatrac_modem");

    Modem d;

    ros::spin();

    return 0;
}