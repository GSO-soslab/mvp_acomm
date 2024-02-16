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

#include "stonefish_driver.hpp"

using goby::glog;
using goby::util::hex_decode;
using goby::util::hex_encode;
using goby::util::number2hex_string;
using namespace goby::util::logger;

goby::acomms::StonefishDriver::StonefishDriver()
{
    // other initialization you can do before you have your goby::acomms::DriverConfig configuration object
}

void goby::acomms::StonefishDriver::startup(const protobuf::DriverConfig& cfg)
{
    driver_cfg_ = cfg;
    // check `driver_cfg_` to your satisfaction and then start the modem physical interface
    if (!driver_cfg_.has_serial_baud())
        driver_cfg_.set_serial_baud(DEFAULT_BAUD);

    glog.is(DEBUG1) && glog << group(glog_out_group())
                            << "StonefishDriver configuration good. Starting modem..." << std::endl;
    modem_start(driver_cfg_);

} // startup

void goby::acomms::StonefishDriver::shutdown()
{
    ModemDriverBase::modem_close();
} // shutdown

void goby::acomms::StonefishDriver::handle_initiate_transmission(
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
            case protobuf::ModemTransmission::DATA: send_data_msg(&transmit_msg_); break;
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
void goby::acomms::StonefishDriver::send_data_msg(protobuf::ModemTransmission* msg)
{
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "\tthis is a DATA transmission"
                                << std::endl;

    msg->set_max_frame_bytes(1);
    msg->set_max_frame_bytes(31);

    const bool is_local_cycle = msg->src() == driver_cfg_.modem_id();
    if(!(is_local_cycle && (msg->frame_size() == 0 || msg->frame(0) == "")))
    {
        // compose the outgoing message
        std::string out;
        out.append(goby::util::as<std::string>(msg->src()));  // src id
        out.append(goby::util::as<std::string>(msg->dest())); // destination id
        out.append(std::to_string(msg->frame_size()));        // packet length
        out.append(msg->frame(0));                            // packet data

        //compress the outgoing message with dccl
        std::string bytes = hex_encode(out);              

        //add the outgoing message to the write queue
        append_to_write_queue(bytes);
    }
    else
    {
        glog.is(DEBUG1) && glog << group(glog_out_group())
                                << "Not initiating transmission because we have no data to send"
                                << std::endl;
    }
}

void goby::acomms::StonefishDriver::do_work()
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

                if(cid == ST_CID_SYS_INFO)
                {
                    CID_SYS_INFO sys_rx;
                    read_sys_info(byte_vec.data(), &sys_rx);
                }
                else if(cid == ST_CID_STATUS)
                {
                    //TODO: Parse status msg
                }
                else if(cid == ST_CID_DAT_RECEIVE)
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
                }
                else if(cid == ST_CID_DAT_SEND)
                {

                    // printf("sent message\n");

                }
                else if(cid == ST_CID_DAT_ERROR)
                {

                    glog.is(QUIET) && glog << group(glog_out_group())
                        << "DAT RECEIVE ERROR" << std::endl;

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

void goby::acomms::StonefishDriver::append_to_write_queue(const std::string &bytes)
{
    // add msg to write queue
    out_.push_back(bytes);

    // try to send a message in the queue
    try_send();
}

void goby::acomms::StonefishDriver::try_send()
{
    if (out_.empty())
    return;

    const std::string bytes = out_.front();

    write(bytes);
}

void goby::acomms::StonefishDriver::write(const std::string &bytes)
{
    protobuf::ModemRaw raw_msg;
    raw_msg.set_raw(bytes);

    signal_raw_outgoing(raw_msg);


}

void goby::acomms::StonefishDriver::convert_to_hex_string(std::ostringstream &op,
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