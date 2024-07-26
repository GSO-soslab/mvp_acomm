
#include "ros/ros.h"

#include <alpha_comms/AcommsRx.h>
#include <alpha_comms/AcommsTx.h>

class Test
{

public:
    Test();

    void loopback_data(const alpha_comms::AcommsTxConstPtr data_msg);

private:

    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_test_rx;
    ros::Subscriber m_test_tx;
};