#include "mvp_acomms.h"

MvpAcomms::MvpAcomms()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    parseEvologicsParams();

    loadGoby();

    m_modem_tx = m_nh->advertise<mvp_acomms::MvpAcommsTx>(config_.type+"/tx", 10);
    m_modem_rx = m_nh->subscribe(config_.type+"/rx", 10, &MvpAcomms::received_data, this);

    m_target_pose = m_nh->advertise<geographic_msgs::GeoPoint>(config_.type + "/pose",10);
    m_target_power = m_nh->advertise<mvp_msgs::Power>(config_.type + "/power", 10);

    m_power_sub = m_nh->subscribe("power_monitor/power", 10, &MvpAcomms::power_callback, this);

    toll_ = m_nh->serviceClient<robot_localization::ToLL>("toLL");

    m_geopose_sub = m_nh->subscribe("odometry/geopose", 10, &MvpAcomms::geopose_callback, this);

    if(config_.type == "modem")
    {
        timer = m_nh->createTimer(ros::Duration(config_.mac_slot_time * 2)), &MvpAcomms::timer_callback, this);
    }
}

void MvpAcomms::parseEvologicsParams()
{
    m_pnh->param<std::string>("type", config_.type, "modem");
    m_pnh->param<int>(config_.type + "_configuration/local_address", config_.local_address, 2);
    m_pnh->param<int>(config_.type + "_configuration/remote_address", config_.remote_address, 1);
    m_pnh_->param<int>("goby/mac_slot_time", config_.mac_slot_time, 10);

    ROS_INFO("Local Address: %i\n", config_.local_address);
    ROS_INFO("Remote Address: %i\n", config_.remote_address);

}

void MvpAcomms::loadGoby()
{
    dccl_->validate<PoseCommand>();
    dccl_->validate<PoseResponse>();
    dccl_->validate<PowerCommand>();
    dccl_->validate<PowerResponse>();

}


/**                                                 
 * @brief the slot that is called back from the driver when a new message is received.
 * 
 * @param data_msg the incoming message
 */
void MvpAcomms::received_data(const mvp_acomms::MvpAcommsRxConstPtr& data_msg)
{
    try
    {
        int dccl_id = dccl_->id_from_encoded(data_msg->data);
        std::string bytes;
        mvp_acomms::MvpAcommsTx out;

        switch(dccl_id)
        {
            case DcclIdMap::POSE_COMMAND_ID:
            {
                PoseCommand pose_cmd;
                dccl_->decode(data_msg->data, &pose_cmd);

                if(pose_cmd.destination() == config_.local_address)
                {

                    dccl_->encode(&bytes, pose_response_);

                    out.data = bytes;
                    out.subbuffer_id = "pose_response";
                    m_modem_tx.publish(out);

                }

                break;
            }
            case DcclIdMap::POSE_RESPONSE_ID:
            {
                PoseResponse pose_resp;
                dccl_->decode(data_msg->data, &pose_resp);

                printf("%s\n", pose_resp.ShortDebugString().c_str());

                if(pose_resp.destination() == config_.local_address)
                {
                    geographic_msgs::GeoPoint geo_msg;

                    geo_msg.latitude = pose_resp.latitude();
                    geo_msg.longitude = pose_resp.longitude();
                    geo_msg.altitude = pose_resp.altitude();

                    m_target_pose.publish(geo_msg);
                }

                break;
            }

            case DcclIdMap::POWER_COMMAND_ID:
            {
                PowerCommand power_command;
                dccl_->decode(data_msg->data, &power_command);

                if(power_command.destination() == config_.local_address)
                {
                    if(good_power_ == true)
                    {
                    dccl_->encode(&bytes, power_response_); 
                
                    out.data = bytes;
                    out.subbuffer_id = "power_response";
                    m_modem_tx.publish(out);
                    }
                
                }

                break;
            }
            case DcclIdMap::POWER_RESPONSE_ID:
            {
                PowerResponse power_resp;
                dccl_->decode(data_msg->data, &power_resp);

                if(power_resp.destination() == config_.local_address)
                {
                    mvp_msgs::Power power;

                    power.voltage = power_resp.battery_voltage();
                    power.current = power_resp.current();


                    m_target_power.publish(power);
                }

            }
            case DcclIdMap::RELATIVE_POSE_COMMAND_ID:
            {
                RelativePoseCommand rel_pose_cmd;
                dccl_->decode(data_msg->data, &rel_pose_cmd);

                if(rel_pose_cmd.destination() == config_.local_address)
                {
                    //do something
                }


            }
            case DcclIdMap::CONTROLLER_STATE_COMMAND_ID:
            {
                ControllerStateCommand controller_state_cmd;
                dccl_->decode(data_msg->data, &controller_state_cmd);

                if(controller_state_cmd.destination() == config_.local_address)
                {
                    if(controller_state_cmd.mode() == ControllerStateCommand_Mode_COMMAND)
                    {

                    }
                    else if(controller_state_cmd.mode() == ControllerStateCommand_Mode_QUERY)
                    {
                        mvp_msgs::GetControlModes srv;
                        m_controller_state_srv.call(srv);

                        ControllerStateResponse controller_state_resp;

                        if(!srv.response.modes.empty())

                        
                            controller_state_resp.set_destination(config_.remote_address);
                            controller_state_resp.set_source(config_.local_address);
                            controller_state_resp.set_time(ros::Time::now().toSec());

                            dccl_->encode(&bytes, controller_state_resp); 
                
                            out.data = bytes;
                            out.subbuffer_id = "controller_state_response";
                            m_modem_tx.publish(out);

                    }

                    
                    
                }


            }
            case DcclIdMap::DIRECT_CONTROL_COMMAND_ID:
            {
                DirectControlCommand direct_control_cmd;
                dccl_->decode(data_msg->data, &direct_control_cmd);

                if(direct_control_cmd.destination() == config_.local_address)
                {
                    //do something
                }


            }
            case DcclIdMap::HELM_STATE_COMMAND_ID:
            {
                HelmStateCommand helm_state_cmd;
                dccl_->decode(data_msg->data, &helm_state_cmd);

                if(helm_state_cmd.destination() == config_.local_address)
                {
                    if(helm_state_cmd.mode() == HelmStateCommand_Mode_QUERY)
                    {
                        mvp_msgs::GetState srv;
                        m_helm_get_state_srv.call(srv);

                        HelmStateResponse helm_state_resp;
                        helm_state_resp.set_source(config_.local_address);
                        helm_state_resp.set_destination(config_.remote_address);
                        helm_state_resp.set_time(ros::Time::now().toSec());

                        HelmStateResponse_HelmState state;

                        if(srv.response.state.mode == "kill"){state = HelmStateResponse_HelmState_KILL;}
                        else if(srv.response.state.mode == "start"){state = HelmStateResponse_HelmState_START;}
                        else if(srv.response.state.mode == "survey_local"){state = HelmStateResponse_HelmState_SURVEY_LOCAL;}
                        else if(srv.response.state.mode == "survey_global"){state = HelmStateResponse_HelmState_SURVEY_GLOBAL;}
                        else if(srv.response.state.mode == "survey_3d"){state = HelmStateResponse_HelmState_SURVEY_3D;}
                        else if(srv.response.state.mode == "direct_control"){state = HelmStateResponse_HelmState_DIRECT_CONTROL;}

                        helm_state_resp.set_helm_state(state);

                        dccl_->encode(&bytes, helm_state_resp);

                        out.data = bytes;
                        out.subbuffer_id = "helm_state_command";
                        m_modem_tx.publish(out);
                            

                    }
                    else if(helm_state_cmd.mode() == HelmStateCommand_Mode_COMMAND)
                    {
                        mvp_msgs::ChangeState srv;
                        
                        if(helm_state_cmd.state() == HelmStateCommand_HelmState_KILL){srv.request.state = "kill";}
                        else if(helm_state_cmd.state() == HelmStateCommand_HelmState_START){srv.request.state = "start";}
                        else if(helm_state_cmd.state() == HelmStateCommand_HelmState_SURVEY_LOCAL){srv.request.state = "survey_local";}
                        else if(helm_state_cmd.state() == HelmStateCommand_HelmState_SURVEY_GLOBAL){srv.request.state = "survey_global";}
                        else if(helm_state_cmd.state() == HelmStateCommand_HelmState_SURVEY_3D){srv.request.state = "survey_3d";}
                        else if(helm_state_cmd.state() == HelmStateCommand_HelmState_DIRECT_CONTROL){srv.request.state = "direct_control";}

                        m_helm_change_state_srv.call(srv);
                    }
                }


            }
            case DcclIdMap::WAYPOINT_COMMAND_ID:
            {
                WaypointCommand waypoint_cmd;
                dccl_->decode(data_msg->data, &waypoint_cmd);

                if(waypoint_cmd.destination() == config_.local_address)
                {
                    //do something
                }


            }
            case DcclIdMap::EXECUTE_WAYPOINTS_ID:
            {
                ControllerStateCommand execute_wpt;
                dccl_->decode(data_msg->data, &execute_wpt);

                if(execute_wpt.destination() == config_.local_address)
                {
                    //do something
                }


            }
            default:
                break;

        }
    }
    catch(...)
    {
        printf("Oops something messed up decoding\n");
    }
}

void MvpAcomms::geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr geopose_msg)
{
    pose_response_.set_source(config_.local_address);
    pose_response_.set_destination(config_.remote_address);
    pose_response_.set_time(geopose_msg->header.stamp.toSec());
    pose_response_.set_latitude(geopose_msg->pose.position.latitude);
    pose_response_.set_longitude(geopose_msg->pose.position.longitude);
    pose_response_.set_altitude(geopose_msg->pose.position.altitude);
    pose_response_.set_quat_x(geopose_msg->pose.orientation.x);
    pose_response_.set_quat_y(geopose_msg->pose.orientation.y);
    pose_response_.set_quat_z(geopose_msg->pose.orientation.z);
    pose_response_.set_quat_w(geopose_msg->pose.orientation.w);

    depth_ =  geopose_msg->pose.position.altitude;
}

void MvpAcomms::odometry_callback(const nav_msgs::OdometryConstPtr odom)
{
    pose_response_.set_source(config_.local_address);
    pose_response_.set_destination(config_.remote_address);
    pose_response_.set_time(odom->header.stamp.toSec());

    robot_localization::ToLL toll;
    toll.request.map_point.x = odom->pose.pose.position.x;
    toll.request.map_point.y = odom->pose.pose.position.y;
    toll.request.map_point.z = odom->pose.pose.position.z;

    toll_.call(toll);

    pose_response_.set_latitude(toll.response.ll_point.latitude);
    pose_response_.set_longitude(toll.response.ll_point.longitude);
    pose_response_.set_altitude(toll.response.ll_point.altitude);
    pose_response_.set_quat_x(odom->pose.pose.orientation.x);
    pose_response_.set_quat_y(odom->pose.pose.orientation.y);
    pose_response_.set_quat_z(odom->pose.pose.orientation.z);
    pose_response_.set_quat_w(odom->pose.pose.orientation.w);

    depth_ =  odom->pose.pose.position.z;
    
}

void MvpAcomms::power_callback(const mvp_msgs::PowerConstPtr power_msg)
{
    good_power_ = true;

    power_response_.set_source(config_.local_address);
    power_response_.set_destination(config_.remote_address);
    power_response_.set_time(power_msg->header.stamp.toSec());
    power_response_.set_battery_voltage(power_msg->voltage);
    //Current Monitor not implemented right now but required in the dccl msg.
    power_response_.set_current(0.0);
}

void MvpAcomms::timer_callback(const ros::TimerEvent& event)
{
    if(depth_ < -1.)
    {
        std::string bytes;
        dccl_->encode(&bytes, pose_response_); 

        printf("Debug String: %s\n", pose_response_.ShortDebugString().c_str() );

        mvp_acomms::MvpAcommsTx out;
        out.data = bytes;
        out.subbuffer_id = "pose_response";
        m_modem_tx.publish(out);

        dccl_->encode(&bytes, power_response_); 

        printf("Debug String: %s\n", power_response_.ShortDebugString().c_str() );

        out.data = bytes;
        out.subbuffer_id = "power_response";
        m_modem_tx.publish(out);
    }
    

}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "alpha_acomms");

    MvpAcomms d;

    ros::spin();

    return 0;
}
