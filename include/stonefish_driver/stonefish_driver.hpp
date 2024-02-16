#ifndef GOBY_ACOMMS_STONEFISH_DRIVER_H
#define GOBY_ACOMMS_STONEFISH_DRIVER_H

#include <map>    // for map
#include <string> // for string

#include "goby/acomms/modemdriver/driver_base.h" // for ModemDriverBase
#include "goby/acomms/protobuf/driver_base.pb.h" // for DriverConfig
#include "goby/acomms/protobuf/modem_message.pb.h"

#include <Stonefish/comms/AcousticModem.h>
#include <Stonefish/comms/USBL.h>

namespace dccl
{
class Codec;
}

namespace goby
{
namespace acomms
{

class StonefishDriver : public ModemDriverBase
{
  public:
    StonefishDriver();
    void startup(const protobuf::DriverConfig& cfg) override;
    void shutdown() override;
    void do_work() override;
    void handle_initiate_transmission(const protobuf::ModemTransmission& m) override;

  private:

    void append_to_write_queue(const std::string &bytes);
    void try_send();
    void write(const std::string &bytes);
    void send_data_msg(protobuf::ModemTransmission* msg);

    protobuf::ModemTransmission transmit_msg_;

    std::deque<std::string> out_;

    enum
    {
        DEFAULT_BAUD = 115200
    };

    protobuf::DriverConfig driver_cfg_; // configuration given to you at launch

    // rest is up to you!
};
} // namespace acomms
} // namespace goby
#endif