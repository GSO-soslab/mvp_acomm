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

#include "seatrac_modem.hpp"

SeaTracModem::SeaTracModem()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    health_publisher = m_nh->advertise<mvp_msgs::ModemHealth>("modem/health", 10);
    pose_publisher = m_nh->advertise<mvp_msgs::ModemPose>("modem/pose", 10);

    local_odom_subscriber = m_nh->subscribe("odometry/filtered/local", 1, &SeaTracModem::f_local_odom_callback, this);

    setup_goby();
    
}

SeaTracModem::~SeaTracModem()
{
    delete driver;
}

void SeaTracModem::setup_goby()
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
    goby::acomms::connect(&driver->signal_receive, this, &SeaTracModem::handle_data_receive);
}

void SeaTracModem::handle_data_receive(const goby::acomms::protobuf::ModemTransmission& data_msg)
{
    std::cout << "got a message: " << data_msg << std::endl;

}

void SeaTracModem::f_local_odom_callback(const nav_msgs::OdometryPtr &msg)
{
    local_x = msg->pose.pose.position.x;
    local_y = msg->pose.pose.position.y;
    local_z = msg->pose.pose.position.z;
    local_p = msg->twist.twist.angular.x;
    local_q = msg->twist.twist.angular.y;
    local_r = msg->twist.twist.angular.z;

    robot_localization::ToLL ser;
    ser.request.map_point.x = local_x;
    ser.request.map_point.y = local_y;
    ser.request.map_point.z = local_z;

    ros::service::call("toll", ser);

    global_lat = ser.response.ll_point.latitude;
    global_long = ser.response.ll_point.longitude;
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac");

    SeaTracModem d;

    ros::spin();

    return 0;
}