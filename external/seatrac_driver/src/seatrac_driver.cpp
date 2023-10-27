#include <cstdint>   // for int32_t
#include <exception> // for exception
#include <list>      // for operator!=
#include <ostream>   // for operator<<
#include <utility>   // for pair, make_pair
#include <vector>    // for vector
#include <bit>

#include <boost/algorithm/string/classification.hpp> // for is_any_ofF
#include <boost/algorithm/string/split.hpp>          // for split
#include <boost/algorithm/string/trim.hpp>           // for trim, trim_copy
#include <boost/algorithm/hex.hpp>

#include "goby/acomms/protobuf/abc_driver.pb.h"    // for Config, Mess...
#include "goby/acomms/protobuf/modem_message.pb.h" // for ModemTransmi...
#include "goby/time/system_clock.h"                // for SystemClock
#include "goby/time/types.h"                       // for MicroTime
#include "goby/util/as.h"                          // for as
#include "goby/util/binary.h"                      // for hex_decode
#include "goby/util/debug_logger.h"                // for glog
#include "goby/util/protobuf/io.h"                 // for operator<<

#include "goby/time/convert.h"

#include "seatrac_driver.h"


using goby::glog;
using goby::util::hex_decode;
using goby::util::hex_encode;
using namespace goby::util::logger;

goby::acomms::SeatracDriver::SeatracDriver()
{
    // other initialization you can do before you have your goby::acomms::DriverConfig configuration object
}

void goby::acomms::SeatracDriver::startup(const protobuf::DriverConfig& cfg)
{
    driver_cfg_ = cfg;
    // check `driver_cfg_` to your satisfaction and then start the modem physical interface
    if (!driver_cfg_.has_serial_baud())
        driver_cfg_.set_serial_baud(DEFAULT_BAUD);

    glog.is(DEBUG1) && glog << group(glog_out_group())
                            << "SeatracDriver configuration good. Starting modem..." << std::endl;
    ModemDriverBase::modem_start(driver_cfg_);

} // startup

void goby::acomms::SeatracDriver::shutdown()
{
    // put the modem in a low power state?
    // ...
    ModemDriverBase::modem_close();
} // shutdown

void goby::acomms::SeatracDriver::handle_initiate_transmission(
    const protobuf::ModemTransmission& orig_msg)
{
    // copy so we can modify
    protobuf::ModemTransmission msg = orig_msg;

    // rate() can be 0 (lowest), 1, 2, 3, 4, or 5 (lowest). Map these integers onto real bit-rates
    // in a meaningful way (on the WHOI Micro-Modem 0 ~= 80 bps, 5 ~= 5000 bps).
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "We were asked to transmit from "
                            << msg.src() << " to " << msg.dest() << std::endl;

    // let's say ABC modem uses 31 byte packet
    msg.set_max_frame_bytes(31);

    // no data given to us, let's ask for some
    if (msg.frame_size() == 0)
        ModemDriverBase::signal_data_request(&msg);

    glog.is(DEBUG1) && glog << group(glog_out_group()) << "Sending these data now: " << hex_decode(msg.frame(0))
                            << std::endl;

    
    CID_DAT_SEND tx;
    tx.dest_id = msg.dest();
    tx.amsgtype = MSG_OWAY;
    std::string hash  = hex_encode(msg.frame(0));
    
    tx.packet_len = hash.length()/2;

    if(tx.packet_len % 2)
    {
        hash.append("00");
        tx.packet_len += 1;
    }

    std::ostringstream os;
    convert_to_hex_string(os, reinterpret_cast<const unsigned char*>(&tx), sizeof(tx));
    os << hash;

    unsigned char buffer[os.str().length()/2];

    std::string bytes = boost::algorithm::unhex(os.str());
    std::copy(bytes.begin(), bytes.end(), buffer);
    int buffLen = sizeof(buffer);
    uint16_t cksumCalc = CRC16(buffer, buffLen);
    uint8_t hibyte = (cksumCalc & 0xff00) >> 8;
    uint8_t lobyte = (cksumCalc & 0xff);
    uint16_t le_cksum = lobyte << 8 | hibyte;
    os << std::hex << le_cksum << "\r\n";
    
    std::ostringstream temp;
    temp <<"#" << os.str();
    
    // let anyone who is interested know
    signal_and_write(temp.str());
} // handle_initiate_transmission

void goby::acomms::SeatracDriver::do_work()
{
    std::string in;
    while (modem_read(&in))
    {
        try
        {
            size_t position = in.find("$");
            in = in.substr(position);
            boost::trim(in); // get whitespace off from either end

            // let others know about the raw feed
            protobuf::ModemRaw raw;
            raw.set_raw(in);
            ModemDriverBase::signal_raw_incoming(raw);

            in.erase(0,1);
            unsigned char buffer[in.length()/2];

            std::string hash = boost::algorithm::unhex(in);
            std::copy(hash.begin(), hash.end(), buffer);

            int buffLen = sizeof(buffer);
            uint16_t cksumRx;
            if(buffLen >= 2) {
                buffLen -= 2;
                cksumRx = (uint16_t)(buffer[buffLen+1] << 8 | buffer[buffLen]);
            }
            else
                cksumRx = 0;

            uint16_t cksumCalc = CRC16(buffer, buffLen);

            glog.is(DEBUG1) && glog << group(glog_out_group())
                            << "RX Checksum: " << std::hex << cksumRx << "\t" << "Calc Checksum: " << std::hex <<cksumCalc << std::endl;

            if(cksumCalc == cksumRx)
            {
                protobuf::ModemTransmission msg;
                uint8_t cid = buffer[0];

                if(cid == ST_CID_SYS_INFO)
                {
                    CID_SYS_INFO sys_rx;
                    read_sys_info(buffer, &sys_rx);
                }
                else if(cid == ST_CID_STATUS)
                {
                    //TODO: Parse status msg
                }
                else if(cid == ST_CID_DAT_RECEIVE)
                {
                    CID_DAT_RX dat_rx;
                    read_dat_msg(buffer, &dat_rx);

                    msg.set_src(dat_rx.aco_fix.local.src_id);
                    msg.set_dest(dat_rx.aco_fix.local.dest_id);
                    msg.set_time_with_units(time::SystemClock::now<time::MicroTime>());
                    msg.set_type(protobuf::ModemTransmission::DATA);
                    std::ostringstream rx;
                    convert_to_hex_string(rx, reinterpret_cast<const unsigned char*>(dat_rx.packet_data), dat_rx.packet_len);
                    std::string rx_string = rx.str();
                    msg.add_frame(hex_decode(rx_string));
                    ModemDriverBase::signal_receive(msg);
                }
                else if(cid == ST_CID_DAT_SEND)
                {

                    printf("sent message\n");

                }
            }
        }
        catch (std::exception& e)
        {
            glog.is(WARN) && glog << "Bad line: " << in << std::endl;
            glog.is(WARN) && glog << "Exception: " << e.what() << std::endl;
        }
    }
    
} // do_work

void goby::acomms::SeatracDriver::signal_and_write(const std::string& raw)
{
    protobuf::ModemRaw raw_msg;
    raw_msg.set_raw(raw);
    ModemDriverBase::signal_raw_outgoing(raw_msg);

    glog.is(QUIET) && glog << group(glog_out_group()) << boost::trim_copy(raw) << std::endl;

    ModemDriverBase::modem_write(raw);
}

void goby::acomms::SeatracDriver::read_sys_info(unsigned char* buffer, struct CID_SYS_INFO *msg)
{
    int offset = 0;
    int size = 0;

    size = sizeof(msg->seconds);
    memcpy(&msg->seconds, buffer, size);
    offset += size;

    size = sizeof(msg->section);
    memcpy(&msg->section, buffer+offset, size);
    offset += size;

    size = sizeof(msg->hardware);
    memcpy(&msg->hardware, buffer+offset, size);
    offset += size;

    size = sizeof(msg->boot_firmware);
    memcpy(&msg->boot_firmware, buffer+offset, size);
    offset += size;

    size = sizeof(msg->main_firmware);
    memcpy(&msg->main_firmware, buffer+offset, size);
    offset += size;

    size = sizeof(msg->board_rev);
    memcpy(&msg->board_rev, buffer+offset, size);
    offset += size;

    size = sizeof(msg->extended_info);
    memcpy(&msg->extended_info, buffer+offset, size);
    offset += size;

    if(msg->section == 1)
    {
        size = sizeof(msg->flags);
        memcpy(&msg->flags, buffer+offset, size);
        offset += size;

        if(msg->flags == 1)
        {
            size = sizeof(msg->reserved);
            memcpy(&msg->reserved, buffer+offset, size);
            offset += size;

            size = sizeof(msg->pressure_sensor);
            memcpy(&msg->pressure_sensor, buffer+offset, size);
        }
    }
}

void goby::acomms::SeatracDriver::read_dat_msg(unsigned char* buffer, struct CID_DAT_RX *msg)
{
    int offset = 0;
    int size = 0;

    size = sizeof(msg->aco_fix.local);
    offset = 1;
    memcpy(&msg->aco_fix.local, buffer + offset, size);
    offset += size;

    if(msg->aco_fix.local.flags & RANGE_VALID)
    {
        size = sizeof(msg->aco_fix.range);
        memcpy(&msg->aco_fix.range, buffer+offset, size);
        offset += size;
    }
    if(msg->aco_fix.local.flags & USBL_VALID)
    {
        size = sizeof(msg->aco_fix.usbl);
        memcpy(&msg->aco_fix.usbl, buffer+offset, size);
        offset += size;
    }
    if(msg->aco_fix.local.flags & POSITION_VALID)
    {
        size = sizeof(msg->aco_fix.position);
        memcpy(&msg->aco_fix.position, buffer+offset, size);
        offset += size;
    }
    if(msg->aco_fix.local.flags & POSITION_ENHANCED)
    {
        //TODO add something to indicate depth is true from the modem
    }
    if(msg->aco_fix.local.flags & POSITION_FLT_ERR)
    {
        //TODO add something to indicate that there could be an error in the position fix calculation
    }

    size = sizeof(msg->ack_flag);
    memcpy(&msg->ack_flag, buffer+offset, size);
    offset += size;

    size = sizeof(msg->packet_len);
    memcpy(&msg->packet_len, buffer+offset, size);
    offset += size;

    size = msg->packet_len;
    memcpy(&msg->packet_data, buffer+offset, size);
    offset += size;
    
    size = sizeof(msg->local_flag);
    memcpy(&msg->local_flag, buffer+offset, size);

}

uint16_t goby::acomms::SeatracDriver::CRC16(uint8_t* buff, uint16_t len)
{	
    uint16_t poly = 0xA001;
    uint16_t crc = 0;
    for(uint16_t b = 0; b < len; b++) {
        uint8_t v = *buff;
        for(uint8_t i = 0; i < 8; i++) {
            if((v & 0x01) ^ (crc & 0x01)) {
                crc >>= 1;
                crc ^= poly;
            }
            else {
                crc >>= 1;
            }
            v >>= 1;
        }
        buff++;
    }
    return crc;
}

void goby::acomms::SeatracDriver::convert_to_hex_string(std::ostringstream &op,
                      const unsigned char* data, int size)
{
    // Format flags
    std::ostream::fmtflags old_flags = op.flags();
 
    // Fill characters
    char old_fill  = op.fill();
    op << std::hex << std::setfill('0');
 
    for (int i = 0; i < size; i++)
    {
        // force output to use hex version of ascii code
        op << std::setw(2) << static_cast<int>(data[i]);
    }
 
    op.flags(old_flags);
    op.fill(old_fill);
}