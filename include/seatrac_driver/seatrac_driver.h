
#ifndef GOBY_ACOMMS_SEATRAC_DRIVER_H
#define GOBY_ACOMMS_SEATRAC_DRIVER_H

#include <map>    // for map
#include <string> // for string

#include "goby/acomms/modemdriver/driver_base.h" // for ModemDriverBase
#include "goby/acomms/protobuf/driver_base.pb.h" // for DriverConfig
#include "goby/acomms/protobuf/modem_message.pb.h"
// #include "goby/acomms/protobuf/seatrac_driver.pb.h"

#include "seatrac_codec.h"

namespace dccl
{
class Codec;
}

namespace goby
{
namespace acomms
{

class SeatracDriver : public ModemDriverBase
{
  public:

    typedef std::function<void(ACOFIX_T)> AcofixCallback;
    AcofixCallback acofix_callback_;

        

    SeatracDriver();
    void startup(const protobuf::DriverConfig& cfg) override;
    void shutdown() override;
    void do_work() override;
    void handle_initiate_transmission(const protobuf::ModemTransmission& m) override;

    void set_usbl_callback(AcofixCallback c) { acofix_callback_  = c;}
    void cid_ping(CID_PING_SEND ping);

  private:

    void read_sys_info(unsigned char* buffer, struct CID_SYS_INFO *msg);
    void read_dat_msg(unsigned char* buffer, struct CID_DAT_RX *msg);
    void convert_to_hex_string(std::ostringstream &op, const unsigned char* data, int size);
    uint16_t CRC16(const uint8_t* buff, uint16_t len);
    bool validate_and_remove_cksum(std::vector<uint8_t> buffer);

    void ciddat(protobuf::ModemTransmission* msg);
    void cid_settings_get();
    void cidnav(protobuf::ModemTransmission *msg);
    void cidecho(protobuf::ModemTransmission* msg);

    void append_to_write_queue(const std::string &bytes);
    void try_send();
    void seatrac_write(const std::string &bytes);



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