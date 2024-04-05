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

    local_odom_sub = m_nh.subscribe("/alpha_rise/odometry/filtered/local", 1, &Modem::f_local_odom_callback, this);

    power_sub = m_nh.subscribe("/alpha_rise/power_monitor/power", 1, &Modem::f_power_callback, this);

    controller_enable_client = m_nh.serviceClient<std_srvs::Empty>("controller/enable");
    controller_disable_client = m_nh.serviceClient<std_srvs::Empty>("controller/disable");
    direct_control_pub = m_nh.advertise<mvp_msgs::ControlProcess>("continuous_command_topic", 10);
    set_state_client = m_nh.serviceClient<mvp_msgs::ChangeState>("helm/change_state");
    get_state_client = m_nh.serviceClient<mvp_msgs::ChangeState>("helm/get_state");
    append_waypoint_pub = m_nh.advertise<geometry_msgs::PolygonStamped>("helm/path_3d/append_waypoints", 10);
    update_waypoint_pub = m_nh.advertise<geometry_msgs::PolygonStamped>("helm/path_3d/update_waypoints", 10);

    setup_goby();

    loop();

    // setup the receive thread
    // std::thread t(std::bind(&Modem::loop, this));
    // t.detach();
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
    int i = 0;

    ros::Rate rate(10);

    PoseCommand pose_test;

    pose_test.set_time(ros::Time::now().toSec());
    pose_test.set_destination(1);
    pose_test.set_source(2);

    // q_manager.push_message(pose_test);

    goby::acomms::DynamicBuffer<std::string>::Value msg;
    msg.subbuffer_id = "PoseCommand";
    msg.push_time.time_since_epoch();
    std::string data;
    pose_test.SerializeToString(&data);
    msg.data = goby::util::hex_encode(data);

    dynamic_buffer.push(msg);

    // loop at 10Hz
    while (ros::ok())
    {

        // every 15 seconds add pose and health to the queue
        // if (i >= 150)
        // {
        //     printf("Pose Out Message: %s\n", pose_test.ShortDebugString().c_str());
        //     printf("Power Out Message: %s\n", power_test.ShortDebugString().c_str());

        //     i = 0;
        //     q_manager.push_message(pose_test);
        //     q_manager.push_message(power_test);

        //     printf("Added Messages to Queue\n");
        // }

        evo_driver.do_work();
        q_manager.do_work();
        mac.do_work();

        ros::spinOnce();


        // i++;


        rate.sleep();
    }
}

/**
 * @brief the goby dccl, mac, queue, and driver are configured and initialized
 *
 */
void Modem::setup_goby()
{
    goby::acomms::bind(evo_driver, q_manager, mac);

    // set the source id of this modem
    our_id = 2;
    dest_id = 1;
    slot_time = 5;

    // Initiate DCCL
    goby::acomms::protobuf::DCCLConfig dccl_cfg;

    // validate messages
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

    // setup the goby queue manager
    setup_queue();

    q_manager_cfg.set_modem_id(our_id);

    goby::acomms::connect(&q_manager.signal_receive, this, &Modem::received_data);
    goby::acomms::connect(&q_manager.signal_ack, this, &Modem::received_ack);

    // Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;
    driver_cfg.set_modem_id(our_id);
    driver_cfg.set_serial_port("/dev/ttyUSB0");
    driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    // Initiate medium access control
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
    goby::glog.add_stream(goby::util::logger::DEBUG1, &std::clog);

    dccl_->set_cfg(dccl_cfg);
    q_manager.set_cfg(q_manager_cfg);
    mac.startup(mac_cfg);
    evo_driver.startup(driver_cfg);
}

/**
 * @brief messages from mvp_messages documentation are loaded and configured into the queue manager.
 *
 */
void Modem::setup_queue()
{
    goby::acomms::protobuf::DynamicBufferConfig dynamic_buffer_cfg;

    dynamic_buffer_cfg.set_max_queue(1);

    dynamic_buffer.create(dest_id,"PoseCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"PoseResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"PowerCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"PowerResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"RelativePoseCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"RelativePoseResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"ControllerStateCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"ControllerStateResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"DirectControlCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"DirectControlResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"HelmStateCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"HelmStateResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"SingleWaypointCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"SingleWaypointResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"MultiWaypointGPSCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"MultiWaypointGPSResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"MultiWaypointXYZCommand", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"MultiWaypointXYZResponse", dynamic_buffer_cfg);
    dynamic_buffer.create(dest_id,"ExecuteWaypoints", dynamic_buffer_cfg);


}

/**
 * @brief the slot that is called back from evo_driver when a new message is received. the incoming
 * message is parsed acording to the mvp_messages documentation.
 *
 * @param data_msg the incoming protobuf message
 */
void Modem::received_data(const google::protobuf::Message &data_msg)
{
    printf("Received Data: %s\n", data_msg.ShortDebugString().c_str());

    std::string msg_type = data_msg.GetTypeName();
    if (msg_type == "PoseCommand")
    {
        // if the received message is a pose command/request
        q_manager.push_message(pose_out);
    }
    else if (msg_type == "HealthCommand")
    {

        q_manager.push_message(power_out);
    }
    else if (msg_type == "RelativePoseCommand")
    {
        RelativePoseCommand rel_pose;
        rel_pose.CopyFrom(data_msg);
    }
    else if (msg_type == "ControllerInfoCommand")
    {
        ControllerStateCommand control_info;
        control_info.CopyFrom(data_msg);

        if (control_info.mode() == ControllerStateCommand_Mode_COMMAND)
        {
            if (control_info.state() == ControllerStateCommand_ControllerState_ENABLE)
            {
                if (ros::service::exists("controller/enable", true))
                {
                    std_srvs::Empty enable;
                    controller_enable_client.call(enable);
                }
            }
            else
            {
                if (ros::service::exists("controller/disable", true))
                {
                    std_srvs::Empty disable;
                    controller_disable_client.call(disable);
                }
            }
        }
        else
        {
            ControllerStateResponse control_info_out;

            control_info_out.set_source(our_id);
            control_info_out.set_destination(dest_id);
            control_info_out.set_time(ros::Time::now().toSec());
            // TODO: Currently no way to ask the controller service for its enable/disable state
            //  control_info_out.set_state(state);
            //  q_manager.push_message(control_info_out);
        }
    }
    else if (msg_type == "DirectControlCommand")
    {
        DirectControlCommand direct_control;
        direct_control.CopyFrom(data_msg);

        mvp_msgs::ControlProcess control_process;

        std::string frame_id;
        switch (direct_control.frame())
        {
        case (DirectControlCommand_Frame_BASE_LINK):
            frame_id = "base_link";
            break;
        case (DirectControlCommand_Frame_ODOM):
            frame_id = "odom";
            break;
        case (DirectControlCommand_Frame_WORLD):
            frame_id = "world";
            break;
        case (DirectControlCommand_Frame_USBL):
            frame_id = "usbl";
            break;
        default:
            frame_id = "world_ned";
        }
        // set header
        control_process.header.frame_id = frame_id;
        control_process.header.stamp = ros::Time::now();

        // set position
        control_process.position.x = direct_control.x();
        control_process.position.y = direct_control.y();
        control_process.position.z = direct_control.z();

        // set orientation
        control_process.orientation.x = direct_control.roll();
        control_process.orientation.y = direct_control.pitch();
        control_process.orientation.z = direct_control.yaw();

        // set linear velocity
        control_process.velocity.x = direct_control.u();
        control_process.velocity.y = direct_control.v();
        control_process.velocity.z = direct_control.w();

        // set angular velocty
        control_process.angular_rate.x = direct_control.p();
        control_process.angular_rate.y = direct_control.q();
        control_process.angular_rate.z = direct_control.r();

        direct_control_pub.publish(control_process);
        direct_control_pub.publish(control_process);

        direct_control_pub.publish(control_process);
    }
    else if (msg_type == "HelmStateCommand")
    {
        HelmStateCommand state_info;
        state_info.CopyFrom(data_msg);

        // Need to take eligible transitions into account.
        mvp_msgs::ChangeState get_state;
        get_state_client.call(get_state);
        std::string current_state = get_state.response.state.name;
        ros::V_string eligible_transitions = get_state.response.state.transitions;
        bool valid_transition = false;

        if (state_info.mode() == HelmStateCommand_Mode_COMMAND)
        {
            std::string cmd_state;

            switch (state_info.state())
            {
            case (HelmStateCommand_HelmState_KILL):
                cmd_state = "kill";
                break;
            case (HelmStateCommand_HelmState_START):
                cmd_state = "start";
                break;
            case (HelmStateCommand_HelmState_SURVEY_LOCAL):
                cmd_state = "survey_local";
                break;
            case (HelmStateCommand_HelmState_SURVEY_GLOBAL):
                cmd_state = "survey_global";
                break;
            case (HelmStateCommand_HelmState_DIRECT_CONTROL):
                cmd_state = "direct_control";
                break;
            case (HelmStateCommand_HelmState_SURVEY_3D):
                cmd_state = "survey_3d";
            default:
                cmd_state = "kill";
            }

            for (ros::V_string::iterator t = eligible_transitions.begin(); t != eligible_transitions.end(); t++)
            {
                if (*t == cmd_state)
                {
                    mvp_msgs::ChangeState set_state;
                    set_state.request.state == cmd_state;
                    set_state_client.call(set_state);
                    break;
                }
            }
        }
        else
        {
            HelmStateResponse state_info_out;
            state_info_out.set_source(our_id);
            state_info_out.set_destination(dest_id);
            state_info_out.set_time(ros::Time::now().toSec());

            for (int i = 0; i < sizeof(HelmStateResponse_HelmState); i++)
            {
                if (current_state == HelmStateResponse_HelmState_Name(HelmStateResponse_HelmState(i)))
                {
                    state_info_out.set_helm_state(HelmStateResponse_HelmState(i));
                    q_manager.push_message(state_info_out);
                    break;
                }
            }
        }
    }
    else if (msg_type == "SingleWaypointCommand")
    {
        SingleWaypointCommand single_waypoint;
        single_waypoint.CopyFrom(data_msg);

        geometry_msgs::Point32 waypoint;

        if (single_waypoint.waypoint_mode() == SingleWaypointCommand_WaypointMode_COMMAND_LATLONG)
        {
            robot_localization::FromLL ser;
            ser.request.ll_point.latitude = single_waypoint.latitude();
            ser.request.ll_point.longitude = single_waypoint.longitude();
            ser.request.ll_point.altitude = 0;

            ros::service::call("fromll", ser);

            waypoint.x = ser.response.map_point.x;
            waypoint.y = ser.response.map_point.y;
            waypoint.z = single_waypoint.z();
        }
        else if (single_waypoint.waypoint_mode() == SingleWaypointCommand_WaypointMode_COMMAND_XYZ)
        {
            waypoint.x = single_waypoint.x();
            waypoint.y = single_waypoint.y();
            waypoint.z = single_waypoint.z();
        }

        // store waypoint in internal buffer, waiting for execute waypoint command to publish
        waypoint_array.polygon.points.push_back(waypoint);
    }
    else if (msg_type == "MultiWaypointGPSCommand")
    {
        MultiWaypointGPSCommand multi_waypoint_gps;
        multi_waypoint_gps.CopyFrom(data_msg);

        robot_localization::FromLL ser;
        geometry_msgs::Point32 waypoint;

        for (int i = 0; i < multi_waypoint_gps.wpt_num_size(); i++)
        {
            ser.request.ll_point.latitude = multi_waypoint_gps.latitude(i);
            ser.request.ll_point.longitude = multi_waypoint_gps.longitude(i);
            ser.request.ll_point.altitude = 0;

            ros::service::call("fromll", ser);

            waypoint.x = ser.response.map_point.x;
            waypoint.y = ser.response.map_point.y;
            waypoint.z = multi_waypoint_gps.z(i);

            std::vector<geometry_msgs::Point32>::iterator position;

            waypoint_array.polygon.points.insert(waypoint_array.polygon.points.begin() + multi_waypoint_gps.wpt_num(i), waypoint);
        }
    }
    else if (msg_type == "MultiWaypointXYZCommand")
    {
        MultiWaypointXYZCommand multi_waypoint_xyz;
        multi_waypoint_xyz.CopyFrom(data_msg);

        geometry_msgs::Point32 waypoint;

        for (int i = 0; i < multi_waypoint_xyz.wpt_num_size(); i++)
        {
            waypoint.x = multi_waypoint_xyz.x(i);
            waypoint.y = multi_waypoint_xyz.y(i);
            waypoint.z = multi_waypoint_xyz.z(i);

            std::vector<geometry_msgs::Point32>::iterator position;

            waypoint_array.polygon.points.insert(waypoint_array.polygon.points.begin() + multi_waypoint_xyz.wpt_num(i), waypoint);
        }
    }
    else if (msg_type == "ExecuteWaypoints")
    {
        ExecuteWaypoints exec_waypoints;
        exec_waypoints.CopyFrom(data_msg);

        waypoint_array.header.frame_id = "world_ned";
        waypoint_array.header.stamp = ros::Time::now();

        if (exec_waypoints.mode() == ExecuteWaypoints_ExecuteMode_APPEND)
        {
            append_waypoint_pub.publish(waypoint_array);
        }
        else if (exec_waypoints.mode() == ExecuteWaypoints_ExecuteMode_UPDATE)
        {
            update_waypoint_pub.publish(waypoint_array);
        }

        waypoint_array.polygon.points.erase(waypoint_array.polygon.points.begin(), waypoint_array.polygon.points.end());
    }
    else
    {
        printf("Msg is not a protobuf msg\n");
    }
}

/**
 * @brief subscriber to vehicles odometry. updating pose_out, a class variable,
 * to be sent over acomms asynchronously
 *
 * @param msg the odometry msg
 */
void Modem::f_local_odom_callback(const nav_msgs::OdometryPtr &msg)
{
    double local_x = msg->pose.pose.position.x;
    double local_y = msg->pose.pose.position.y;
    double local_z = msg->pose.pose.position.z;

    double x_rot = msg->pose.pose.orientation.x;
    double y_rot = msg->pose.pose.orientation.y;
    double z_rot = msg->pose.pose.orientation.z;
    double w_rot = msg->pose.pose.orientation.w;

    robot_localization::ToLL ser;
    ser.request.map_point.x = local_x;
    ser.request.map_point.y = local_y;
    ser.request.map_point.z = local_z;

    ros::service::call("/alpha_rise/toLL", ser);

    double global_lat = ser.response.ll_point.latitude;
    double global_long = ser.response.ll_point.longitude;

    pose_out.set_source(our_id);
    pose_out.set_destination(dest_id);
    pose_out.set_time(msg->header.stamp.toSec());
    pose_out.set_latitude(global_lat);
    pose_out.set_longitude(global_long);
    pose_out.set_x(local_x);
    pose_out.set_y(local_y);
    pose_out.set_z(local_z);
    pose_out.set_quat_x(x_rot);
    pose_out.set_quat_y(y_rot);
    pose_out.set_quat_z(z_rot);
    pose_out.set_quat_w(w_rot);
}

/**
 * @brief time synced subscriber to voltage and current. updating power_out, a class variable,
 * to be sent over acomms asynchronously
 *
 * @param voltage the voltage msg
 * @param current the current msg
 */
void Modem::f_power_callback(const mvp_msgs::PowerPtr &power)
{
    power_out.set_source(our_id);
    power_out.set_destination(dest_id);
    power_out.set_time(power->header.stamp.toSec());
    power_out.set_battery_voltage(power->voltage);
    power_out.set_current(power->current);
   
}

/**
 * @brief TODO: Enable the ack protocol to confirm messages are being routed correctly
 *
 * @param ack_message
 * @param original_message
 */
void Modem::received_ack(const goby::acomms::protobuf::ModemTransmission &ack_message,
                                const google::protobuf::Message &original_message)
{

    std::cout << ack_message.src() << " acknowledged receiving message: " << original_message.ShortDebugString() << std::endl;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "seatrac_modem");

    Modem d;

    ros::spin();

    return 0;
}