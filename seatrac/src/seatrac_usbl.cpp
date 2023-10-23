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
    
    ros::Rate rate(0.1);

    while(ros::ok())
    {
        Pose msg;
        msg.set_destination(1);
        msg.set_cmd_resp(true);
        q_manager.push_message(msg);

        rate.sleep();
    }
}

SeaTracUSBL::~SeaTracUSBL()
{
    delete driver;
}

void SeaTracUSBL::setup_goby()
{
    // set the serial port given on the command line
    cfg.set_serial_port("/dev/ttyUSB0");
    // set the source id of this modem
    uint32_t our_id = goby::util::as<uint32_t>(beacon_id);
    cfg.set_modem_id(our_id);
    qcfg.set_modem_id(our_id);
    cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    qcfg.add_message_entry()->set_protobuf_name("Pose");
    qcfg.add_message_entry()->set_protobuf_name("Health");
    qcfg.add_message_entry()->set_protobuf_name("RelativePose");
    qcfg.add_message_entry()->set_protobuf_name("ControllerInfo");
    qcfg.add_message_entry()->set_protobuf_name("DirectControl");
    qcfg.add_message_entry()->set_protobuf_name("StateInfo");
    qcfg.add_message_entry()->set_protobuf_name("SingleWaypoint");
    qcfg.add_message_entry()->set_protobuf_name("MultiWaypoint");
    qcfg.add_message_entry()->set_protobuf_name("ExecuteWaypoint");

    q_manager.set_cfg(qcfg);

    goby::glog.set_name("usbl");

    if(console_debug)
    {
        goby::glog.add_stream(goby::util::logger::DEBUG2, &std::clog);
    }
    
    std::cout << "Starting seatrac driver: " << std::endl;
    driver = new goby::acomms::SeatracDriver;
    goby::acomms::connect(&driver->signal_receive, this, &SeaTracUSBL::handle_data_receive);
}

void SeaTracUSBL::handle_data_receive(const goby::acomms::protobuf::ModemTransmission& data_msg)
{
    std::cout << "got a message: " << data_msg << std::endl;

//     int idx = data_msg.frame_size();
//     for(int i=0; i<idx; i++)
//     {
//         data_msg.frame(idx);
//     }
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac");

    SeaTracUSBL d;

    ros::spin();

    return 0;
}