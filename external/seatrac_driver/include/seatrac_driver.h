
#ifndef GOBY_ACOMMS_SEATRAC_DRIVER_H
#define GOBY_ACOMMS_SEATRAC_DRIVER_H

#include <map>    // for map
#include <string> // for string

#include "goby/acomms/modemdriver/driver_base.h" // for ModemDriverBase
#include "goby/acomms/protobuf/driver_base.pb.h" // for DriverConfig
#include "seatrac_codec.h"

namespace goby
{
namespace acomms
{
namespace protobuf
{
class ModemTransmission;
} // namespace protobuf

/// \brief provides an API to the imaginary ABC modem (as an example how to write drivers)
/// \ingroup acomms_api
///
class SeatracDriver : public ModemDriverBase
{
  public:
    SeatracDriver();
    void startup(const protobuf::DriverConfig& cfg) override;
    void shutdown() override;
    void do_work() override;
    void handle_initiate_transmission(const protobuf::ModemTransmission& m) override;

  private:
    void parse_in(const std::string& in, std::map<std::string, std::string>* out);
    void signal_and_write(const std::string& raw);
    void read_sys_info(unsigned char* buffer, struct CID_SYS_INFO *msg);
    void read_acoustic_msg(unsigned char* buffer, struct CID_XCVR_RX *msg);
    void read_dat_msg(unsigned char* buffer, struct CID_DAT_RX *msg);
    void convert_to_hex_string(std::ostringstream &op, const unsigned char* data, int size);
    uint16_t CRC16(uint8_t* buff, uint16_t len);

  private:
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