
#include "ros/ros.h"

#include "common.h"

#include <goby/acomms/dccl.h>

#include <alpha_acomms/AcommsRx.h>
#include <alpha_acomms/AcommsTx.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mvp_msgs/Power.h>
#include <mvp_msgs/GetControlModes.h>
#include <mvp_msgs/GetState.h>
#include <mvp_msgs/GetStates.h>
#include <mvp_msgs/ChangeState.h>



#include "proto/goby_msgs.pb.h"


class AlphaAcomms
{

public:
    AlphaAcomms();

    void parseEvologicsParams();

    void received_data(const alpha_acomms::AcommsRxConstPtr data_msg);

    void geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr geopose_msg);

    void power_callback(const mvp_msgs::PowerConstPtr power_msg);

    void timer_callback(const ros::TimerEvent &event);

private:

    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_modem_tx;
    ros::Subscriber m_modem_rx;

    ros::Publisher m_target_pose;
    ros::Publisher m_target_power;

    ros::Subscriber m_odom_sub;
    ros::Subscriber m_power_sub;
    ros::ServiceClient m_controller_state_srv;
    ros::ServiceClient m_helm_get_state_srv;
    ros::ServiceClient m_helm_get_states_srv;
    ros::ServiceClient m_helm_change_state_srv;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();


    PoseResponse pose_response_;
    PowerResponse power_response_;

    struct Config
    {
        std::string type;
        int local_address;
        int remote_address;
    };

    Config config_;

    ros::Timer timer;
};