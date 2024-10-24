
#include "ros/ros.h"

#include <goby/acomms/dccl.h>

//ros messages
#include <mvp_acomms/MvpAcommsRx.h>
#include <mvp_acomms/MvpAcommsTx.h>
#include "common.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mvp_msgs/Power.h>

//ros 
#include <mvp_msgs/GetControlModes.h>
#include <mvp_msgs/GetState.h>
#include <mvp_msgs/GetStates.h>
#include <mvp_msgs/ChangeState.h>

#include "robot_localization/ToLL.h"

#include "mvp_msgs.pb.h"


class MvpAcomms
{

public:
    MvpAcomms();

    void parseEvologicsParams();

    void loadGoby();

    void received_data(const mvp_acomms::MvpAcommsRxConstPtr& data_msg);

    void geopose_callback(const geographic_msgs::GeoPoseStampedConstPtr geopose_msg);

    void odometry_callback(const nav_msgs::OdometryConstPtr);

    void power_callback(const mvp_msgs::PowerConstPtr power_msg);

    void timer_callback(const ros::TimerEvent &event);

    void test_timer_callback(const ros::TimerEvent &event);

private:

    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_modem_tx;
    ros::Subscriber m_modem_rx;

    ros::Publisher m_target_pose;
    ros::Publisher m_target_power;

    ros::Subscriber m_geopose_sub;
    ros::Subscriber m_power_sub;
    ros::ServiceClient m_controller_state_srv;
    ros::ServiceClient m_helm_get_state_srv;
    ros::ServiceClient m_helm_get_states_srv;
    ros::ServiceClient m_helm_change_state_srv;

    ros::ServiceClient toll_;

    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();


    PoseResponse pose_response_;
    PowerResponse power_response_;

    bool good_power_ = false;

    struct Config
    {
        std::string type;
        int local_address;
        int remote_address;
        int mac_slot_time;
        float valid_depth;
    };

    Config config_;

    ros::Timer timer;

    float depth_;
};