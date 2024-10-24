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
#include "goby/acomms/modemdriver/driver_exception.h"

#include "seatrac_driver.h"


using goby::glog;
using goby::util::hex_decode;
using goby::util::hex_encode;
using goby::util::number2hex_string;
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
    modem_start(driver_cfg_);

    cid_settings_get();

} // startup

void goby::acomms::SeatracDriver::shutdown()
{
    ModemDriverBase::modem_close();
} // shutdown

void goby::acomms::SeatracDriver::handle_initiate_transmission(
    const protobuf::ModemTransmission& msg)
{
    // copy so we can modify
    transmit_msg_.CopyFrom(msg);

    try
    {
        glog.is(QUIET) && glog << group(glog_out_group()) << "We were asked to transmit from "
                                << msg.src() << " to " << msg.dest() << std::endl;

        signal_modify_transmission(&transmit_msg_);

        switch(transmit_msg_.type())
        {
            case protobuf::ModemTransmission::DATA: ciddat(&transmit_msg_); break;
            // case protobuf::ModemTransmission::DRIVER_SPECIFIC:
            // {
            //     switch (transmit_msg_.GetExtension(seatrac::protobuf::transmission).type())
            //     {
            //         case seatrac::protobuf::SEATRAC_TWO_WAY_PING: cidping(&transmit_msg_); break;
            //         case seatrac::protobuf::SEATRAC_NAVIGATION: cidnav(&transmit_msg_); break;
            //         case seatrac::protobuf::SEATRAC_ECHO: cidecho(&transmit_msg_); break;   
            //         default:
            //             glog.is(DEBUG1) &&
            //                     glog << group(glog_out_group()) << warn
            //                         << "Not initiating transmission because we were given an invalid "
            //                             "DRIVER_SPECIFIC transmission type for the Micro-Modem:"
            //                         << transmit_msg_ << std::endl;
            //             break;
            //     }
            // }
        }
    }

    catch (ModemDriverException& e)
    {
        glog.is(DEBUG1) && glog << group(glog_out_group()) << warn
                                << "Failed to initiate transmission: " << e.what() << std::endl;
    }

} // handle_initiate_transmission


/**
 * @brief data packet to be sent to the modem. 31 max bytes. currently MSG_OWAY.
 * 
 * @param msg the protobuf message to send
 */
void goby::acomms::SeatracDriver::ciddat(protobuf::ModemTransmission* msg)
{
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "\tthis is a DATA transmission"
                                << std::endl;

    msg->set_max_frame_bytes(1);
    msg->set_max_frame_bytes(31);

    const bool is_local_cycle = msg->src() == driver_cfg_.modem_id();
    if(!(is_local_cycle && (msg->frame_size() == 0 || msg->frame(0) == "")))
    {
        std::string out;
        out.append("96"); //command id of cid_dat_send
        out.append(goby::util::as<std::string>(msg->dest())); // destination id
        out.append(std::to_string(MSG_OWAY)); // msg_type
        out.append(std::to_string(msg->frame_size())); // packet_length
        out.append(msg->frame(0));
        std::string bytes = hex_encode(out);

        append_to_write_queue(bytes);
    }
    else
    {
        glog.is(DEBUG1) && glog << group(glog_out_group())
                                << "Not initiating transmission because we have no data to send"
                                << std::endl;
    }
}

void goby::acomms::SeatracDriver::cid_settings_get()
{
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "\tthis is a SETTINGS_GET transmission"
                            << std::endl;

    std::string out;
    out.append("15");

    append_to_write_queue(out);
}

void goby::acomms::SeatracDriver::cid_ping(CID_PING_SEND ping)
{
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "\tthis is a PING transmission"
                                << std::endl;

    std::string out;
    out.append("64"); //command id of cid_ping_send
    out.append(goby::util::as<std::string>(ping.dest_id)); // destination id
    out.append(std::to_string(ping.msg_type)); // msg_type

    std::string bytes = hex_encode(out);

    append_to_write_queue(bytes);
}

void goby::acomms::SeatracDriver::cidnav(protobuf::ModemTransmission* msg)
{

}

void goby::acomms::SeatracDriver::cidecho(protobuf::ModemTransmission* msg)
{

}

void goby::acomms::SeatracDriver::do_work()
{
    std::string in;
    while (modem_read(&in))
    {
        try
        {
            //search the string for the dollar sign, indicating the start of a msg
            size_t position = in.find("$");
            //split the string off where a msg starts
            in = in.substr(position);
            // get whitespace off from either end
            boost::trim(in); 

            //let others know about the raw feed
            protobuf::ModemRaw raw;
            raw.set_raw(in);
            ModemDriverBase::signal_raw_incoming(raw);

            //remove the dollar sign
            in.erase(0,1);

            //decode the hex string
            std::string hash = boost::algorithm::unhex(in);

            //convert the string type to a vector of bytes
            std::vector<uint8_t> byte_vec(hash.begin(), hash.end());


            //validate the checksum
            bool cksum_valid = validate_and_remove_cksum(byte_vec);

            if(cksum_valid)
            {
                //initialize generic received msg
                protobuf::ModemTransmission msg;

                //look at what the command id is
                uint8_t cid = byte_vec[0];

                switch(cid)
                {
                    case ST_CID_SYS_INFO:
                    {
                        CID_SYS_INFO sys_rx;
                        read_sys_info(byte_vec.data(), &sys_rx);

                        break;
                    }
                    case ST_CID_STATUS:
                    {

                        break;
                    }
                    case ST_CID_DAT_RECEIVE:
                    {
                        CID_DAT_RX dat_rx;
                        read_dat_msg(byte_vec.data(), &dat_rx);

                        msg.set_src(dat_rx.aco_fix.local.src_id);
                        msg.set_dest(dat_rx.aco_fix.local.dest_id);
                        msg.set_time_with_units(time::SystemClock::now<time::MicroTime>());
                        msg.set_type(protobuf::ModemTransmission::DATA);
                        std::ostringstream rx;
                        convert_to_hex_string(rx, reinterpret_cast<const unsigned char*>(dat_rx.packet_data), dat_rx.packet_len);
                        std::string rx_string = rx.str();
                        msg.add_frame(hex_decode(rx_string));
                        ModemDriverBase::signal_receive(msg);

                        break;
                    }
                    case ST_CID_DAT_SEND:
                    {

                        break;
                    }
                    case ST_CID_PING_SEND:
                    {

                        break;
                    }
                    case ST_CID_PING_RESP:
                    {
                        CID_PING_RESP ping_resp;
                        memcpy(&ping_resp, byte_vec.data(), byte_vec.size());

                        acofix_callback_(ping_resp.aco_fix);

                        break;
                    }
                    case ST_CID_SETTINGS_GET:
                    {
                        CID_SETTINGS_GET settings;
                        memcpy(&settings, byte_vec.data(), byte_vec.size());

                        printf("XCVR Beacon ID: %i\n", settings.xcvr_beacon_id);

                        break;
                    }
                    case ST_CID_DAT_ERROR:
                    {
                        glog.is(QUIET) && glog << group(glog_out_group())
                            << "DAT RECEIVE ERROR" << std::endl;

                        break;
                    }

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

void goby::acomms::SeatracDriver::append_to_write_queue(const std::string &bytes)
{
    out_.push_back(bytes);
    try_send();
}

void goby::acomms::SeatracDriver::try_send()
{
    if (out_.empty())
    return;

    const std::string bytes = out_.front();

    seatrac_write(bytes);
}

void goby::acomms::SeatracDriver::seatrac_write(const std::string &bytes)
{
    protobuf::ModemRaw raw_msg;
    raw_msg.set_raw(bytes);

    signal_raw_outgoing(raw_msg);

    uint16_t cksum = CRC16(reinterpret_cast<const uint8_t*>(raw_msg.raw().c_str()), sizeof(raw_msg.raw().c_str()));

    printf("%s\n", raw_msg.ShortDebugString().c_str());  

    modem_write("#" + raw_msg.raw() + "*" + number2hex_string(cksum));
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

uint16_t goby::acomms::SeatracDriver::CRC16(const uint8_t* buff, uint16_t len)
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

bool goby::acomms::SeatracDriver::validate_and_remove_cksum(std::vector<uint8_t> buffer)
{

    int buffLen = buffer.size();
    uint16_t cksumRx;
    if(buffLen >= 2) {
        buffLen -= 2;
        cksumRx = (uint16_t)(buffer[buffLen+1] << 8 | buffer[buffLen]);
    }
    else
        cksumRx = 0;

    uint16_t cksumCalc = CRC16(buffer.data(), buffLen);

    glog.is(DEBUG1) && glog << group(glog_out_group())
                    << "RX Checksum: " << std::hex << cksumRx << "\t" << "Calc Checksum: " << std::hex <<cksumCalc << std::endl;

    if(cksumCalc == cksumRx){buffer.resize(buffer.size()-2); return true;}
    else{return false;}
}