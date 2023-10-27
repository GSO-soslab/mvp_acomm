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

#include "seatrac_usbl.hpp"

SeaTracUSBL::SeaTracUSBL()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    health_publisher = m_nh->advertise<mvp_msgs::ModemHealth>("usbl/health", 10);
    pose_publisher = m_nh->advertise<mvp_msgs::ModemPose>("usbl/pose", 10);

    setup_goby();
    
    ros::Rate rate(10);

    // Pose p_msg;
    // p_msg.set_cmd_resp(true);
    // q_manager.push_message(p_msg);

    Health h_msg;
    h_msg.set_destination(1);
    h_msg.set_cmd_resp(true);
    q_manager.push_message(h_msg);
    int i = 0;

    while(ros::ok())
    {
        i++;
        st_driver.do_work();
        q_manager.do_work();
        mac.do_work();
        
                // // send another transmission every 10 seconds
        if (!(i % 200))
            i = 0;
            // q_manager.push_message(p_msg);
            q_manager.push_message(h_msg);

        usleep(100000);
    }
}

SeaTracUSBL::~SeaTracUSBL()
{

}

void SeaTracUSBL::setup_goby()
{
    goby::acomms::bind(st_driver, q_manager, mac);

    // set the source id of this modem
    uint32_t our_id = 15;
    uint32_t dest_id = 1;
    uint32_t slot_time = 5;

    //Initiate DCCL
    goby::acomms::protobuf::DCCLConfig dccl_cfg;

    //validate messages
    dccl_->validate<Pose>();
    dccl_->validate<Health>();
    dccl_->validate<RelativePose>();
    dccl_->validate<ControllerInfo>();
    dccl_->validate<DirectControl>();
    dccl_->validate<StateInfo>();
    dccl_->validate<SingleWaypoint>();
    dccl_->validate<MultiWaypoint>();
    dccl_->validate<ExecuteWaypoints>();

    //setup protobuf messages
    setup_msgs();
    
    q_manager_cfg.set_modem_id(our_id);

    goby::acomms::connect(&q_manager.signal_receive, this, &SeaTracUSBL::received_data);
    goby::acomms::connect(&q_manager.signal_ack, this, &SeaTracUSBL::received_ack);
    //Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;
    driver_cfg.set_modem_id(our_id);
    driver_cfg.set_serial_port("/dev/ttyUSB1");
    driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    //Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(our_id);

    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(our_id);
    my_slot.set_dest(dest_id);
    my_slot.set_rate(0);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    my_slot.set_slot_seconds(slot_time);

    goby::acomms::protobuf::ModemTransmission buddy_slot;
    buddy_slot.set_src(dest_id);
    buddy_slot.set_dest(our_id);
    buddy_slot.set_rate(0);
    buddy_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    buddy_slot.set_slot_seconds(slot_time);

    if (our_id < dest_id)
    {
        mac_cfg.add_slot()->CopyFrom(my_slot);
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
    }
    else
    {
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
        mac_cfg.add_slot()->CopyFrom(my_slot);
    }


    goby::glog.set_name("usbl");
    goby::glog.add_stream(goby::util::logger::QUIET, &std::clog);

    dccl_->set_cfg(dccl_cfg);
    q_manager.set_cfg(q_manager_cfg);   
    mac.startup(mac_cfg);
    st_driver.startup(driver_cfg);
}

void SeaTracUSBL::setup_msgs()
{
    //setup pose msg
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_pose = q_manager_cfg.add_message_entry();
    q_entry_pose->set_protobuf_name("Pose");
    q_entry_pose->set_ack(false);
    q_entry_pose->set_max_queue(1);

    //setup health msg
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_health = q_manager_cfg.add_message_entry();
    q_entry_health->set_protobuf_name("Health");
    q_entry_health->set_ack(false);
    q_entry_health->set_max_queue(1);

    //setup relative pose
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_rel_pose = q_manager_cfg.add_message_entry();
    q_entry_rel_pose->set_protobuf_name("RelativePose");;
    q_entry_rel_pose->set_ack(false);
    q_entry_rel_pose->set_max_queue(1);

    //setup controller info
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_controll = q_manager_cfg.add_message_entry();
    q_entry_controll->set_protobuf_name("ControllerInfo");;
    q_entry_controll->set_ack(false);
    q_entry_controll->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_direct = q_manager_cfg.add_message_entry();
    q_entry_direct->set_protobuf_name("DirectControl");;
    q_entry_direct->set_ack(false);
    q_entry_direct->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_state = q_manager_cfg.add_message_entry();
    q_entry_state->set_protobuf_name("StateInfo");;
    q_entry_state->set_ack(false);
    q_entry_state->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_single_wpt = q_manager_cfg.add_message_entry();
    q_entry_single_wpt->set_protobuf_name("SingleWaypoint");;
    q_entry_single_wpt->set_ack(false);
    q_entry_single_wpt->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_multi_wpt = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt->set_protobuf_name("MultiWaypoint");;
    q_entry_multi_wpt->set_ack(false);
    q_entry_multi_wpt->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_exe_wpt = q_manager_cfg.add_message_entry();
    q_entry_exe_wpt->set_protobuf_name("ExecuteWaypoints");;
    q_entry_exe_wpt->set_ack(false);
    q_entry_exe_wpt->set_max_queue(1);


}

void SeaTracUSBL::received_data(const google::protobuf::Message& message_in)
{
    std::cout << "Received Data: " << message_in << std::endl;
}

void SeaTracUSBL::received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message,
                  const google::protobuf::Message& original_message)
{
    std::cout << ack_message.src() << " acknowledged receiving message: " << original_message << std::endl;
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac");

    SeaTracUSBL d;

    ros::spin();

    return 0;
}