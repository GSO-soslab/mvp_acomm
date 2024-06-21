#include "alpha_acomms.h"





AlphaAcomms::AlphaAcomms()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_modem_tx = m_nh->advertise<alpha_acomms::AcommsTx>("modem/tx", 10);
    m_modem_rx = m_nh->subscribe("modem/rx", 10, &AlphaAcomms::received_data, this);



}


/**
 * @brief the slot that is called back from the driver when a new message is received.
 * 
 * @param data_msg the incoming message
 */
void AlphaAcomms::received_data(const alpha_acomms::AcommsRxConstPtr data_msg)
{

    int dccl_id = dccl_->id_from_encoded(data_msg->data);
    std::string bytes;
    alpha_acomms::AcommsTx out;

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
                //out.subbuffer_id = ;
                m_modem_tx.publish(out);

            }

            break;
        }
        case DcclIdMap::POWER_COMMAND_ID:
        {
            PowerCommand power_command;
            dccl_->decode(data_msg->data, &power_command);

            if(power_command.destination() == config_.local_address)
            {
                dccl_->encode(&bytes, power_response_); 
            
                out.data = bytes;
                //out.subbuffer_id = ;
                m_modem_tx.publish(out);
            
            }

            break;
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
                    //out.subbuffer_id = ;
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

void AlphaAcomms::geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr geopose_msg)
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
    pose_response_.set_quat_z(geopose_msg->pose.orientation.z);
}

void AlphaAcomms::power_callback(const mvp_msgs::PowerConstPtr power_msg)
{
    power_response_.set_source(config_.local_address);
    power_response_.set_destination(config_.remote_address);
    power_response_.set_time(power_msg->header.stamp.toSec());
    power_response_.set_battery_voltage(power_msg->voltage);
    //Current Monitor not implemented right now but required in the dccl msg.
    power_response_.set_current(0);
}