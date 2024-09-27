#include <string>
#include <sstream>
#include <vector>

struct AtType
{
    std::string command;

    std::vector<std::string> fields;
};
struct XYZ
{
    float x;
    float y;
    float z;
};

struct ENU
{
    float e;
    float n;
    float u;
};

struct RPY
{
    float roll;
    float pitch;
    float yaw;
};

struct UsbllongMsg
{
    float current_time;
    float measurement_time;
    int remote_address;
    XYZ xyz;
    ENU enu;
    RPY rpy;
    float propogation_time;
    int rssi;
    int integrity;
    float accuracy;
};

class ATsentence
{
public:
    ATsentence();
    std::string encode(const AtType &at_sentance);
    AtType decode(const std::string &);

private:
};
