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

    local_odom_subscriber = m_nh->subscribe("odometry/filtered/local", 1, &SeaTracModem::f_local_odom_callback, this);
    voltage_sub = m_nh->subscribe("power/voltage", 1, &SeaTracModem::f_voltage_callback, this);
    controller_enable_client = m_nh->serviceClient<std_srvs::Empty>("controller/enable");
    controller_disable_client = m_nh->serviceClient<std_srvs::Empty>("controller/disable");
    direct_control_pub = m_nh->advertise<mvp_msgs::ControlProcess>("continuous_command_topic", 10);

    setup_goby();

    while(ros::ok())
    {
        st_driver.do_work();
        q_manager.do_work();
        mac.do_work();
        usleep(100000);
    }
    
}

SeaTracModem::~SeaTracModem()
{

}

void SeaTracModem::setup_goby()
{
    goby::acomms::bind(st_driver, q_manager, mac);

    // set the source id of this modem
    uint32_t our_id = 1;
    uint32_t dest_id = 15;
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

    goby::acomms::connect(&q_manager.signal_receive, this, &SeaTracModem::received_data);
    goby::acomms::connect(&q_manager.signal_ack, this, &SeaTracModem::received_ack);
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


    goby::glog.set_name("modem");
    goby::glog.add_stream(goby::util::logger::DEBUG2, &std::clog);

    dccl_->set_cfg(dccl_cfg);
    q_manager.set_cfg(q_manager_cfg);   
    mac.startup(mac_cfg);
    st_driver.startup(driver_cfg);
}

void SeaTracModem::setup_msgs()
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

    //setup multi waypoint
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_multi_wpt = q_manager_cfg.add_message_entry();
    q_entry_multi_wpt->set_protobuf_name("MultiWaypoint");;
    q_entry_multi_wpt->set_ack(false);
    q_entry_multi_wpt->set_max_queue(1);

    //setup execute waypoints
    goby::acomms::protobuf::QueuedMessageEntry* q_entry_exe_wpt = q_manager_cfg.add_message_entry();
    q_entry_exe_wpt->set_protobuf_name("ExecuteWaypoints");;
    q_entry_exe_wpt->set_ack(false);
    q_entry_exe_wpt->set_max_queue(1);


}

void SeaTracModem::received_data(const google::protobuf::Message& data_msg)
{
    std::string msg_type =  data_msg.GetTypeName();
    if(msg_type == "Pose")
    {
        Pose pose;
        pose.CopyFrom(data_msg);

        //if the received message is a pose command/request
        if(pose.cmd_resp())
        {
            Pose pose_out;
            pose_out.set_destination(15);
            pose_out.set_cmd_resp(false);
            pose_out.set_time(ros::Time::now().toSec());
            pose_out.set_latitude(global_lat);
            pose_out.set_longitude(global_long);
            pose_out.set_local_x(std::round(local_x));
            pose_out.set_local_y(std::round(local_y));
            pose_out.set_local_z(std::round(local_z));
            pose_out.set_x_rot(x_rot);
            pose_out.set_y_rot(y_rot);
            pose_out.set_z_rot(z_rot);
            pose_out.set_w_rot(w_rot);
            q_manager.push_message(pose_out);

        }
    }
    else if(msg_type == "Health")
    {
        Health health;
        health.CopyFrom(data_msg);
        if(health.cmd_resp())
        {
            Health health_out;
            health_out.set_destination(15);
            health_out.set_cmd_resp(false);
            health_out.set_time(ros::Time::now().toSec());
            health_out.set_batt_volt(voltage);
            health_out.set_current(current);
            q_manager.push_message(health_out);
        }

    }
    else if(msg_type == "RelativePose")
    {
        RelativePose rel_pose;
        rel_pose.CopyFrom(data_msg);
    }
    else if(msg_type == "ControllerInfo")
    {
        ControllerInfo control_info;
        control_info.CopyFrom(data_msg);

        if(control_info.setget())
        {
            if(control_info.state())
            {
                if(ros::service::exists("controller/enable", true))
                {
                    std_srvs::Empty enable;
                    controller_enable_client.call(enable);
                }
            }
            else
            {
                if(ros::service::exists("controller/disable", true))
                {
                    std_srvs::Empty disable;
                    controller_disable_client.call(disable);
                }
            }
        }
        else
        {
            ControllerInfo control_info_out;

            control_info_out.set_destination(15);
            control_info_out.set_time(ros::Time::now().toSec());
            control_info_out.set_setget(false);
            //TODO: Currently no way to ask the controller service for its enable/disable state
            // control_info_out.set_state(state);
        }
    }
    else if(msg_type == "DirectControl")
    {
        DirectControl direct_control;
        direct_control.CopyFrom(data_msg);
        if(direct_control.setget())
        {
            mvp_msgs::ControlProcess control_process;
            
            std::string frame_id;
            switch(direct_control.frame())
            {
            case(DirectControl_Frame_BASE_LINK):
                frame_id = "base_link";
                break;
            case(DirectControl_Frame_ODOM):
                frame_id = "odom";
                break;
            case(DirectControl_Frame_WORLD):
                frame_id = "world";
                break;
            case(DirectControl_Frame_USBL):
                frame_id = "usbl";
                break;
            default:
                frame_id = "base_link";
            }
            //set header
            control_process.header.frame_id = frame_id;
            control_process.header.stamp = ros::Time::now();

            //set position
            control_process.position.x = direct_control.set_x();
            control_process.position.y = direct_control.set_y();
            control_process.position.z = direct_control.set_z();

            //set orientation
            control_process.orientation.x = direct_control.set_roll();
            control_process.orientation.y = direct_control.set_pitch();
            control_process.orientation.z = direct_control.set_yaw();

            //set linear velocity
            control_process.velocity.x = direct_control.set_u();
            control_process.velocity.y = direct_control.set_v();
            control_process.velocity.z = direct_control.set_w();

            //set angular velocty
            control_process.angular_rate.x = direct_control.set_p();
            control_process.angular_rate.y = direct_control.set_q();
            control_process.angular_rate.z = direct_control.set_r();

            direct_control_pub.publish(control_process);

        }
    }
    else if(msg_type == "StateInfo")
    {
        StateInfo state_info;
        state_info.CopyFrom(data_msg);

        if(state_info.setget())
        {
            
        }
        else
        {
            
        }
    }
    else if(msg_type == "SingleWaypoint")
    {
        SingleWaypoint single_waypoint;
        single_waypoint.CopyFrom(data_msg);
    }
    else if(msg_type == "MultiWaypoint")
    {
        MultiWaypoint multi_waypoint;
        multi_waypoint.CopyFrom(data_msg);
    }
    else if(msg_type == "ExecuteWaypoints")
    {
        ExecuteWaypoints exec_waypoints;
        exec_waypoints.CopyFrom(data_msg);
    }
    
}

void SeaTracModem::received_ack(const goby::acomms::protobuf::ModemTransmission& ack_message,
                  const google::protobuf::Message& original_message)
{
    std::cout << ack_message.src() << " acknowledged receiving message: " << original_message << std::endl;
}

void SeaTracModem::f_local_odom_callback(const nav_msgs::OdometryPtr &msg)
{
    local_x = msg->pose.pose.position.x;
    local_y = msg->pose.pose.position.y;
    local_z = msg->pose.pose.position.z;

    x_rot = msg->pose.pose.orientation.x;
    y_rot = msg->pose.pose.orientation.y;
    z_rot = msg->pose.pose.orientation.z;
    w_rot = msg->pose.pose.orientation.w;

    robot_localization::ToLL ser;
    ser.request.map_point.x = local_x;
    ser.request.map_point.y = local_y;
    ser.request.map_point.z = local_z;

    ros::service::call("toll", ser);

    global_lat = ser.response.ll_point.latitude;
    global_long = ser.response.ll_point.longitude;
}

void SeaTracModem::f_voltage_callback(const mvp_msgs::Float64Stamped &msg)
{
    voltage = msg.data;
}

void SeaTracModem::f_current_callback(const mvp_msgs::Float64Stamped &msg)
{
    current = msg.data;
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "seatrac");

    SeaTracModem d;

    ros::spin();

    return 0;
}