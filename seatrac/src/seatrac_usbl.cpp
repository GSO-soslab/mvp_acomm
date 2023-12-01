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


/**
 * @brief Construct a new Sea Trac USBL:: SeaTrac USBL object
 * 
 */
SeaTracUSBL::SeaTracUSBL()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    odom_pub = m_nh->advertise<nav_msgs::Odometry>("comms/odometry/filtered/local", 10);
    health_pub = m_nh->advertise<mvp_msgs::VehicleStatus>("comms/health", 10);

    pose_server = m_nh->advertiseService
        <mvp_msgs::CommsPose::Request, 
        mvp_msgs::CommsPose::Response>
    (
        "comms/request_pose",
        std::bind(
            &SeaTracUSBL::f_cb_srv_request_pose,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    health_server = m_nh->advertiseService
        <mvp_msgs::CommsHealth::Request,
        mvp_msgs::CommsHealth::Response>
    (
        "comms/request_health",
        std::bind(
            &SeaTracUSBL::f_cb_srv_request_health,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )    
    );

    relative_pose_server = m_nh->advertiseService
        <mvp_msgs::CommsRelativePose::Request,
        mvp_msgs::CommsRelativePose::Response>
    (
        "comms/request_relative_pose",
        std::bind(
            &SeaTracUSBL::f_cb_srv_request_relative_pose,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    controller_info_server = m_nh->advertiseService
        <mvp_msgs::CommsControllerInfo::Request,
        mvp_msgs::CommsControllerInfo::Response>
    (
        "comms/reuest_controller_info",
        std::bind(
            &SeaTracUSBL::f_cb_srv_controller_info,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    direct_control_server = m_nh->advertiseService
        <mvp_msgs::CommsDirectControl::Request,
        mvp_msgs::CommsDirectControl::Response>
    (
        "comms/direct_control",
        std::bind(
            &SeaTracUSBL::f_cb_srv_direct_control,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    state_info_server = m_nh->advertiseService
        <mvp_msgs::CommsStateInfo::Request,
        mvp_msgs::CommsStateInfo::Response>
    (
        "comms/state_info",
        std::bind(
            &SeaTracUSBL::f_cb_srv_state_info,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    single_waypoint_server = m_nh->advertiseService
        <mvp_msgs::CommsSingleWaypoint::Request,
        mvp_msgs::CommsSingleWaypoint::Response>
    (
        "comms/single_waypoint",
        std::bind(
            &SeaTracUSBL::f_cb_srv_single_waypoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    multi_waypoint_gps_server = m_nh->advertiseService
        <mvp_msgs::CommsMultiWaypointGPS::Request,
        mvp_msgs::CommsMultiWaypointGPS::Response>
    (
        "comms/multi_waypoint_gps",
        std::bind(
            &SeaTracUSBL::f_cb_srv_multi_waypoint_gps,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    multi_waypoint_xyz_server = m_nh->advertiseService
        <mvp_msgs::CommsMultiWaypointXYZ::Request,
        mvp_msgs::CommsMultiWaypointXYZ::Response>
    (
        "comms/multi_waypoint_xyz",
        std::bind(
            &SeaTracUSBL::f_cb_srv_multi_waypoint_xyz,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    execute_waypoints_server = m_nh->advertiseService
        <mvp_msgs::CommsExecuteWaypoint::Request,
        mvp_msgs::CommsExecuteWaypoint::Response>
    (
        "comms/execute_waypoint",
        std::bind(
            &SeaTracUSBL::f_cb_srv_execute_waypoints,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    setup_goby();

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
 * @brief Destroy the SeaTracUSBL:: SeaTracUSBL object
 * 
 */
SeaTracUSBL::~SeaTracUSBL()
{

}

/**
 * @brief the goby dccl, mac, queue, and driver are configured and initialized
 * 
 */
void SeaTracUSBL::setup_goby()
{
    goby::acomms::bind(st_driver, q_manager, mac);

    // set the source id of this modem
    our_id = 15;
    dest_id = 1;
    slot_time = 5;

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
    dccl_->validate<MultiWaypointGPS>();
    dccl_->validate<MultiWaypointXYZ>();
    dccl_->validate<ExecuteWaypoints>();


    //setup the goby queue manager
    setup_queue();
    
    q_manager_cfg.set_modem_id(our_id);

    goby::acomms::connect(&q_manager.signal_receive, this, &SeaTracUSBL::received_data);
    goby::acomms::connect(&q_manager.signal_ack, this, &SeaTracUSBL::received_ack);
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
    goby::glog.add_stream(goby::util::logger::DEBUG2, &std::clog);

    dccl_->set_cfg(dccl_cfg);
    q_manager.set_cfg(q_manager_cfg);   
    mac.startup(mac_cfg);
    st_driver.startup(driver_cfg);
}

/**
 * @brief messages from mvp_messages documentation are loaded and configured into the queue manager.
 * 
 */
void SeaTracUSBL::setup_queue()
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
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_control = q_manager_cfg.add_message_entry();
    q_entry_control->set_protobuf_name("ControllerInfo");;
    q_entry_control->set_ack(false);
    q_entry_control->set_max_queue(1);

    //setup direct control
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_direct = q_manager_cfg.add_message_entry();
    q_entry_direct->set_protobuf_name("DirectControl");;
    q_entry_direct->set_ack(false);
    q_entry_direct->set_max_queue(1);

    //setup state info
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_state = q_manager_cfg.add_message_entry();
    q_entry_state->set_protobuf_name("StateInfo");;
    q_entry_state->set_ack(false);
    q_entry_state->set_max_queue(1);

    //setup single waypoint
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_single_wpt = q_manager_cfg.add_message_entry();
    q_entry_single_wpt->set_protobuf_name("SingleWaypoint");;
    q_entry_single_wpt->set_ack(false);
    q_entry_single_wpt->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_multi_wpt_gps = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_gps->set_protobuf_name("MultiWaypointGPS");;
    q_entry_multi_wpt_gps->set_ack(false);
    q_entry_multi_wpt_gps->set_max_queue(1);

    goby::acomms::protobuf::QueuedMessageEntry* q_entry_multi_wpt_xyz = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt_xyz->set_protobuf_name("MultiWaypointXYZ");;
    q_entry_multi_wpt_xyz->set_ack(false);
    q_entry_multi_wpt_xyz->set_max_queue(1);

    //setup multi waypoint
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_exe_wpt = q_manager_cfg.add_message_entry();
    q_entry_exe_wpt->set_protobuf_name("ExecuteWaypoints");;
    q_entry_exe_wpt->set_ack(false);
    q_entry_exe_wpt->set_max_queue(1);


}

/**
 * @brief the slot that is called back from seatrac_driver when a new message is received. the incoming 
 * message is parsed acording to the mvp_messages documentation.
 * 
 * @param data_msg the incoming protobuf message
 */
void SeaTracUSBL::received_data(const google::protobuf::Message& data_msg)
{
    std::string msg_type =  data_msg.GetTypeName();
    if(msg_type == "Pose")
    {
        Pose pose;
        pose.CopyFrom(data_msg);

        //if the received message is a pose command/request
        if(!pose.cmd_resp())
        {
            nav_msgs::Odometry odom;

            odom.header.stamp = ros::Time::now();

            odom.header.frame_id = "alpha_rise/odom";

            odom.pose.pose.position.x = pose.local_x();
            odom.pose.pose.position.y = pose.local_y();
            odom.pose.pose.position.z = pose.local_z();
            
            odom.pose.pose.orientation.x = pose.x_rot();
            odom.pose.pose.orientation.y = pose.y_rot();
            odom.pose.pose.orientation.z = pose.z_rot();
            odom.pose.pose.orientation.w = pose.w_rot();


            odom_pub.publish(odom);

        }
    }
    else if(msg_type == "Health")
    {
        Health health;
        health.CopyFrom(data_msg);
        if(!health.cmd_resp())
        {
            mvp_msgs::VehicleStatus status;

            status.current = health.current();
            status.voltage = health.batt_volt();

            health_pub.publish(status);
        }
    }  
}

/**
 * @brief TODO: Enable the ack protocol to confirm messages are being routed correctly
 * 
 * @param ack_message 
 * @param original_message 
 */
void SeaTracUSBL::received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message,
                  const google::protobuf::Message& original_message)
{
    
    std::cout << ack_message.src() << " acknowledged receiving message: " << original_message.ShortDebugString() << std::endl;
}

bool SeaTracUSBL::f_cb_srv_request_pose(mvp_msgs::CommsPose::Request &request, mvp_msgs::CommsPose::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_request_health(mvp_msgs::CommsHealth::Request &request, mvp_msgs::CommsHealth::Response &response)
{
    
    return true;
}

bool SeaTracUSBL::f_cb_srv_request_relative_pose(mvp_msgs::CommsRelativePose::Request &request, mvp_msgs::CommsRelativePose::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_controller_info(mvp_msgs::CommsControllerInfo::Request &request, mvp_msgs::CommsControllerInfo::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_direct_control(mvp_msgs::CommsDirectControl::Request &request, mvp_msgs::CommsDirectControl::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_state_info(mvp_msgs::CommsStateInfo::Request &request, mvp_msgs::CommsStateInfo::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_single_waypoint(mvp_msgs::CommsSingleWaypoint::Request &request, mvp_msgs::CommsSingleWaypoint::Response &response)
{

    return true;
}

bool SeaTracUSBL::f_cb_srv_multi_waypoint_gps(mvp_msgs::CommsMultiWaypointGPS::Request &request, mvp_msgs::CommsMultiWaypointGPS::Response &response)
{

    return true;
}
bool SeaTracUSBL::f_cb_srv_multi_waypoint_xyz(mvp_msgs::CommsMultiWaypointXYZ::Request &request, mvp_msgs::CommsMultiWaypointXYZ::Response &response)
{

    return true;
}
bool SeaTracUSBL::f_cb_srv_execute_waypoints(mvp_msgs::CommsExecuteWaypoint::Request &request, mvp_msgs::CommsExecuteWaypoint::Response &response)
{

    return true;
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac_usbl");

    SeaTracUSBL d;

    ros::spin();

    return 0;
}