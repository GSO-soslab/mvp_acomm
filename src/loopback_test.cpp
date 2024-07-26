#include "loopback_test.h"

Test::Test()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_test_tx = m_nh->subscribe("test/tx", 10, &Test::loopback_data, this);
    m_test_rx = m_nh->advertise<alpha_comms::AcommsRx>("test/rx", 10);
}

void Test::loopback_data(const alpha_comms::AcommsTxConstPtr msg)
{
    

    alpha_comms::AcommsRx out;

    out.data = msg->data;

    printf("Loopback data: %s\n", msg->data.c_str());

    m_test_rx.publish(out);

}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "test_comms");

    Test d;

    ros::spin();

    return 0;
}
