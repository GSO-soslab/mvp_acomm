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

#include "usbl.hpp"

using goby::util::as;
/**
 * @brief Construct a USBL object
 * 
 */
USBL::USBL()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    odom_pub = m_nh->advertise<nav_msgs::Odometry>("comms/odometry/filtered/local", 10);
    power_pub = m_nh->advertise<mvp_msgs::Power>("comms/power", 10);

    pose_server = m_nh->advertiseService
        <alpha_acomms::CommsPose::Request, 
        alpha_acomms::CommsPose::Response>
    (
        "comms/request_pose",
        std::bind(
            &USBL::f_cb_srv_request_pose,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    power_server = m_nh->advertiseService
        <alpha_acomms::CommsPower::Request,
        alpha_acomms::CommsPower::Response>
    (
        "comms/request_power",
        std::bind(
            &USBL::f_cb_srv_request_power,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )    
    );

    relative_pose_server = m_nh->advertiseService
        <alpha_acomms::CommsRelativePose::Request,
        alpha_acomms::CommsRelativePose::Response>
    (
        "comms/request_relative_pose",
        std::bind(
            &USBL::f_cb_srv_request_relative_pose,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    controller_info_server = m_nh->advertiseService
        <alpha_acomms::CommsControllerInfo::Request,
        alpha_acomms::CommsControllerInfo::Response>
    (
        "comms/reuest_controller_info",
        std::bind(
            &USBL::f_cb_srv_controller_info,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    direct_control_server = m_nh->advertiseService
        <alpha_acomms::CommsDirectControl::Request,
        alpha_acomms::CommsDirectControl::Response>
    (
        "comms/direct_control",
        std::bind(
            &USBL::f_cb_srv_direct_control,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    state_info_server = m_nh->advertiseService
        <alpha_acomms::CommsStateInfo::Request,
        alpha_acomms::CommsStateInfo::Response>
    (
        "comms/state_info",
        std::bind(
            &USBL::f_cb_srv_state_info,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    single_waypoint_server = m_nh->advertiseService
        <alpha_acomms::CommsSingleWaypoint::Request,
        alpha_acomms::CommsSingleWaypoint::Response>
    (
        "comms/single_waypoint",
        std::bind(
            &USBL::f_cb_srv_single_waypoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    multi_waypoint_gps_server = m_nh->advertiseService
        <alpha_acomms::CommsMultiWaypointGPS::Request,
        alpha_acomms::CommsMultiWaypointGPS::Response>
    (
        "comms/multi_waypoint_gps",
        std::bind(
            &USBL::f_cb_srv_multi_waypoint_gps,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    multi_waypoint_xyz_server = m_nh->advertiseService
        <alpha_acomms::CommsMultiWaypointXYZ::Request,
        alpha_acomms::CommsMultiWaypointXYZ::Response>
    (
        "comms/multi_waypoint_xyz",
        std::bind(
            &USBL::f_cb_srv_multi_waypoint_xyz,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    execute_waypoints_server = m_nh->advertiseService
        <alpha_acomms::CommsExecuteWaypoint::Request,
        alpha_acomms::CommsExecuteWaypoint::Response>
    (
        "comms/execute_waypoint",
        std::bind(
            &USBL::f_cb_srv_execute_waypoints,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    setup_goby();

        // setup the receive thread
    std::thread t(std::bind(&USBL::loop, this));
    t.detach();


    
}

/**
 * @brief Destroy the USBL:: USBL object
 * 
 */
USBL::~USBL()
{

}

void USBL::loop()
{
        //loop at 10Hz 
    while(ros::ok())
    {
        st_driver.do_work();
        q_manager.do_work();
        mac.do_work();

        usleep(100000);
    }

}

/**
 * @brief the goby dccl, mac, queue, and driver are configured and initialized
 * 
 */
void USBL::setup_goby()
{
    goby::acomms::bind(st_driver, q_manager, mac);

    // set the source id of this modem
    our_id = 15;
    dest_id = 1;
    slot_time = 5;

    //Initiate DCCL
    goby::acomms::protobuf::DCCLConfig dccl_cfg;

    //validate messages
    dccl_->validate<PoseCommand>();
    dccl_->validate<PoseResponse>();
    dccl_->validate<PowerCommand>();
    dccl_->validate<PowerResponse>();
    dccl_->validate<RelativePoseCommand>();
    dccl_->validate<RelativePoseResponse>();
    dccl_->validate<ControllerStateCommand>();
    dccl_->validate<ControllerStateResponse>();
    dccl_->validate<DirectControlCommand>();
    dccl_->validate<HelmStateCommand>();
    dccl_->validate<HelmStateResponse>();
    dccl_->validate<SingleWaypointCommand>();
    dccl_->validate<SingleWaypointResponse>();
    dccl_->validate<MultiWaypointGPSCommand>();
    dccl_->validate<MultiWaypointGPSResponse>();
    dccl_->validate<MultiWaypointXYZCommand>();
    dccl_->validate<MultiWaypointXYZResponse>();
    dccl_->validate<ExecuteWaypoints>();


    //setup the goby queue manager
    setup_queue();
    
    q_manager_cfg.set_modem_id(our_id);

    goby::acomms::connect(&q_manager.signal_receive, this, &USBL::received_data);
    goby::acomms::connect(&q_manager.signal_ack, this, &USBL::received_ack);
    //Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;
    driver_cfg.set_modem_id(our_id);
    driver_cfg.set_serial_port("/dev/ttyUSB0");
    driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    //Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(our_id);

    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(our_id);
    my_slot.set_dest(dest_id);
    my_slot.set_rate(0);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DRIVER_SPECIFIC);

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

/**
 * @brief messages from mvp_messages documentation are loaded and configured into the queue manager.
 * 
 */
void USBL::setup_queue()
{
    // setup pose msg
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_pose_cmd = q_manager_cfg.add_message_entry();
    q_entry_pose_cmd->set_protobuf_name("PoseCommand");
    q_entry_pose_cmd->set_ack(false);
    q_entry_pose_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_pose_resp = q_manager_cfg.add_message_entry();
    q_entry_pose_resp->set_protobuf_name("PoseResponse");
    q_entry_pose_resp->set_ack(false);
    q_entry_pose_resp->set_max_queue(1);

    // setup health msg
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_power_cmd = q_manager_cfg.add_message_entry();
    q_entry_power_cmd->set_protobuf_name("PowerCommand");
    q_entry_power_cmd->set_ack(false);
    q_entry_power_cmd->set_max_queue(1);

    // setup health msg
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_power_resp = q_manager_cfg.add_message_entry();
    q_entry_power_resp->set_protobuf_name("PowerResponse");
    q_entry_power_resp->set_ack(false);
    q_entry_power_resp->set_max_queue(1);

    // setup relative pose
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_rel_pose_cmd = q_manager_cfg.add_message_entry();
    q_entry_rel_pose_cmd->set_protobuf_name("RelativePoseCommand");
    q_entry_rel_pose_cmd->set_ack(false);
    q_entry_rel_pose_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_rel_pose_resp = q_manager_cfg.add_message_entry();
    q_entry_rel_pose_resp->set_protobuf_name("RelativePoseResponse");
    q_entry_rel_pose_resp->set_ack(false);
    q_entry_rel_pose_resp->set_max_queue(1);

    // setup controller info
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_control_state_cmd = q_manager_cfg.add_message_entry();
    q_entry_control_state_cmd->set_protobuf_name("ControllerStateCommand");
    q_entry_control_state_cmd->set_ack(false);
    q_entry_control_state_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_control_state_resp = q_manager_cfg.add_message_entry();
    q_entry_control_state_resp->set_protobuf_name("ControllerStateResponse");
    q_entry_control_state_resp->set_ack(false);
    q_entry_control_state_resp->set_max_queue(1);

    // setup direct control
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_direct = q_manager_cfg.add_message_entry();
    q_entry_direct->set_protobuf_name("DirectControlCommand");
    q_entry_direct->set_ack(false);
    q_entry_direct->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_helm_state_cmd = q_manager_cfg.add_message_entry();
    q_entry_helm_state_cmd->set_protobuf_name("HelmStateCommand");
    q_entry_helm_state_cmd->set_ack(false);
    q_entry_helm_state_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_helm_state_resp = q_manager_cfg.add_message_entry();
    q_entry_helm_state_resp->set_protobuf_name("HelmStateResponse");
    q_entry_helm_state_resp->set_ack(false);
    q_entry_helm_state_resp->set_max_queue(1);

    // setup single waypoint
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_single_wpt_cmd = q_manager_cfg.add_message_entry();
    q_entry_single_wpt_cmd->set_protobuf_name("SingleWaypointCommand");
    q_entry_single_wpt_cmd->set_ack(false);
    q_entry_single_wpt_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_single_wpt_response = q_manager_cfg.add_message_entry();
    q_entry_single_wpt_response->set_protobuf_name("SingleWaypointResponse");
    q_entry_single_wpt_response->set_ack(false);
    q_entry_single_wpt_response->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_multi_wpt_gps_cmd = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_gps_cmd->set_protobuf_name("MultiWaypointGPSCommand");
    q_entry_multi_wpt_gps_cmd->set_ack(false);
    q_entry_multi_wpt_gps_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_multi_wpt_gps_resp = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_gps_resp->set_protobuf_name("MultiWaypointGPSResponse");
    q_entry_multi_wpt_gps_resp->set_ack(false);
    q_entry_multi_wpt_gps_resp->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_multi_wpt_xyz_cmd = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_xyz_cmd->set_protobuf_name("MultiWaypointXYZCommand");
    q_entry_multi_wpt_xyz_cmd->set_ack(false);
    q_entry_multi_wpt_xyz_cmd->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry *q_entry_multi_wpt_xyz_resp = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_xyz_resp->set_protobuf_name("MultiWaypointXYZResponse");
    q_entry_multi_wpt_xyz_resp->set_ack(false);
    q_entry_multi_wpt_xyz_resp->set_max_queue(1);

    // setup multi waypoint
    goby::acomms::protobuf::QueuedMessageEntry *q_entry_exe_wpt = q_manager_cfg.add_message_entry();
    q_entry_exe_wpt->set_protobuf_name("ExecuteWaypoints");
    q_entry_exe_wpt->set_ack(false);
    q_entry_exe_wpt->set_max_queue(1);


}

/**
 * @brief the slot that is called back from seatrac_driver when a new message is received. the incoming 
 * message is parsed acording to the mvp_messages documentation.
 * 
 * @param data_msg the incoming protobuf message
 */
void USBL::received_data(const google::protobuf::Message& data_msg)
{
    std::string msg_type =  data_msg.GetTypeName();
    printf("Received %s: %s\n", msg_type.c_str(), data_msg.ShortDebugString().c_str());

    if(msg_type == "PoseResponse")
    {
        PoseResponse pose;
        pose.CopyFrom(data_msg);

        //if the received message is a pose command/request
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();

        odom.header.frame_id = "alpha_rise/odom";

        odom.pose.pose.position.x = pose.x();
        odom.pose.pose.position.y = pose.y();
        odom.pose.pose.position.z = pose.z();
        
        odom.pose.pose.orientation.x = pose.quat_x();
        odom.pose.pose.orientation.y = pose.quat_y();
        odom.pose.pose.orientation.z = pose.quat_z();
        odom.pose.pose.orientation.w = pose.quat_w();


        odom_pub.publish(odom);

    }
    else if(msg_type == "PowerResponse")
    {
        PowerResponse power_response;
        power_response.CopyFrom(data_msg);

        mvp_msgs::Power power_out;

        power_out.current = power_response.current();
        power_out.voltage = power_response.battery_voltage();

        power_pub.publish(power_out);
        
    }  
}

/**
 * @brief TODO: Enable the ack protocol to confirm messages are being routed correctly
 * 
 * @param ack_message 
 * @param original_message 
 */
void USBL::received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message,
                  const google::protobuf::Message& original_message)
{
    
    std::cout << ack_message.src() << " acknowledged receiving message: " << original_message.ShortDebugString() << std::endl;
}


/**
 * @brief Callback to add pose request to the queue manager
 * 
 * @param request 
 * @param response 
 * @return true 
 * @return false 
 */
bool USBL::f_cb_srv_request_pose(alpha_acomms::CommsPose::Request &request, alpha_acomms::CommsPose::Response &response)
{
    PoseCommand pose_cmd;

    pose_cmd.set_source(our_id);
    pose_cmd.set_destination(dest_id);
    pose_cmd.set_time(ros::Time::now().toSec()); 

    q_manager.push_message(pose_cmd);

    return true;
}

/**
 * @brief Callback for health service call
 * 
 * @param request 
 * @param response 
 * @return true 
 * @return false 
 */
bool USBL::f_cb_srv_request_power(alpha_acomms::CommsPower::Request &request, alpha_acomms::CommsPower::Response &response)
{
    PowerCommand power_request;

    power_request.set_source(our_id);
    power_request.set_destination(dest_id);
    power_request.set_time(ros::Time::now().toSec());
    
    q_manager.push_message(power_request);
    
    return true;
}

bool USBL::f_cb_srv_request_relative_pose(alpha_acomms::CommsRelativePose::Request &request, alpha_acomms::CommsRelativePose::Response &response)
{
    RelativePoseCommand rel_pose_command;

    rel_pose_command.set_source(our_id);
    rel_pose_command.set_destination(dest_id);
    rel_pose_command.set_time(ros::Time::now().toSec());
    // rel_pose_command.set_parent();
    // rel_pose_command.set_child();

    return true;
}

bool USBL::f_cb_srv_controller_info(alpha_acomms::CommsControllerInfo::Request &request, alpha_acomms::CommsControllerInfo::Response &response)
{

    return true;
}

bool USBL::f_cb_srv_direct_control(alpha_acomms::CommsDirectControl::Request &request, alpha_acomms::CommsDirectControl::Response &response)
{

    return true;
}

bool USBL::f_cb_srv_state_info(alpha_acomms::CommsStateInfo::Request &request, alpha_acomms::CommsStateInfo::Response &response)
{

    return true;
}

bool USBL::f_cb_srv_single_waypoint(alpha_acomms::CommsSingleWaypoint::Request &request, alpha_acomms::CommsSingleWaypoint::Response &response)
{

    return true;
}

bool USBL::f_cb_srv_multi_waypoint_gps(alpha_acomms::CommsMultiWaypointGPS::Request &request, alpha_acomms::CommsMultiWaypointGPS::Response &response)
{

    return true;
}
bool USBL::f_cb_srv_multi_waypoint_xyz(alpha_acomms::CommsMultiWaypointXYZ::Request &request, alpha_acomms::CommsMultiWaypointXYZ::Response &response)
{

    return true;
}
bool USBL::f_cb_srv_execute_waypoints(alpha_acomms::CommsExecuteWaypoint::Request &request, alpha_acomms::CommsExecuteWaypoint::Response &response)
{

    return true;
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac_usbl");

    USBL d;

    ros::spin();

    return 0;
}