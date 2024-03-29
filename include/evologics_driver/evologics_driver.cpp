
#include <algorithm>   // for copy, max
#include <cerrno>      // for errno
#include <chrono>      // for seconds
#include <cmath>       // for abs
#include <cstdlib>     // for abs
#include <cstring>     // for strerror
#include <iterator>    // for ostrea...
#include <list>        // for operat...
#include <locale>      // for locale
#include <memory>      // for allocator
#include <sstream>     // for basic_...
#include <stdexcept>   // for out_of...
#include <sys/ioctl.h> // for ioctl
#include <unistd.h>    // for usleep
#include <utility>     // for pair
#include <vector>      // for vector

#include <boost/algorithm/string/case_conv.hpp>      // for to_upp...
#include <boost/algorithm/string/classification.hpp> // for is_any...
#include <boost/algorithm/string/split.hpp>          // for split
#include <boost/algorithm/string/trim.hpp>           // for trim
#include <boost/date_time/date.hpp>                  // for date<>...
#include <boost/date_time/gregorian/gregorian.hpp>   // for date
#include <boost/date_time/posix_time/ptime.hpp>      // for ptime
#include <boost/date_time/time.hpp>                  // for base_t...
#include <boost/signals2/signal.hpp>                 // for signal
#include <dccl/binary.h>                             // for b64_de...
#include <dccl/codec.h>                              // for Codec
#include <dccl/common.h>                             // for operat...

#include "goby/acomms/acomms_constants.h"                // for BROADC...
#include "goby/acomms/protobuf/modem_driver_status.pb.h" // for ModemD...
#include "goby/time/convert.h"                           // for System...
#include "goby/time/types.h"                             // for MicroTime
#include "goby/util/as.h"                                // for as
#include "goby/util/binary.h"                            // for hex_de...
#include "goby/util/debug_logger/flex_ostream.h"         // for FlexOs...
#include "goby/util/debug_logger/flex_ostreambuf.h"      // for DEBUG1
#include "goby/util/debug_logger/logger_manipulators.h"  // for operat...
#include "goby/util/debug_logger/term_color.h"           // for magenta
#include "goby/util/linebasedcomms/interface.h"          // for LineBa...
#include "goby/util/linebasedcomms/serial_client.h"      // for Serial...
#include "goby/util/protobuf/io.h"                       // for operat...

#include "goby/acomms/modemdriver/driver_exception.h" // for ModemD...

#include "evologics_driver.h"
// #include "at_sentence.h"

using goby::glog;
using goby::util::as;
using goby::util::hex_decode;
using goby::util::hex_encode;
// using goby::util::ATSentence;
using namespace goby::util::tcolor;
using namespace goby::util::logger;
using namespace goby::util::logger_lock;

const std::string goby::acomms::EvologicsDriver::SERIAL_DELIMITER = "\r";
const std::string goby::acomms::EvologicsDriver::ETHERNET_DELIMITER = "\n";

goby::acomms::EvologicsDriver::EvologicsDriver()
{
    initialize_talkers();
}

void goby::acomms::EvologicsDriver::initialize_talkers()
{
    sentence_id_map_ = {
        {"OK", OK}, {"DELIVEREDIM", DELIVEREDIM}, {"RECVIM", RECVIM}, {"SENDSTART", SENDSTART},
        {"SENDEND", SENDEND}, {"RECVSTART", RECVSTART}, {"RECVEND", RECVEND}, {"FAILEDIM", FAILEDIM}, 
        {"CANCELLEDIM", CANCELLEDIM}, {"CANCELLEDIMS", CANCELLEDIMS}, {"CANCELLEDPBM", CANCELLEDPBM},
        {"EXPIREDIMS", EXPIREDIMS}, {"BITRATE", BITRATE}, {"SRCLEVEL", SRCLEVEL}, {"STATUS", STATUS}, 
        {"PHYON", PHYON}, {"PHYOFF", PHYOFF}, {"RECVFAILED", RECVFAILED}, {"RECVSRV", RECVSRV}, {"RADDR", RADDR},
        {"USBLPHYP", USBLPHYP}, {"USBLPHYD", USBLPHYD}, {"USBLLONG", USBLLLONG}, {"USBLANGLES", USBLANGLES},
        {"ERROR", ERROR}, {"RECVPBM", RECVPBM}, {"RECVIMS", RECVIMS}};
}

goby::acomms::EvologicsDriver::~EvologicsDriver() = default;

void goby::acomms::EvologicsDriver::startup(const protobuf::DriverConfig& cfg)
{

    glog.is(DEBUG1) && glog << group(glog_out_group()) << "Goby Evologics driver starting up."
                            << std::endl;

    if (startup_done_)
    {
        glog.is(DEBUG1) && glog << group(glog_out_group())
                                << " ... driver is already started, not restarting." << std::endl;
        return;
    }

    // store a copy for us later
    driver_cfg_ = cfg;


    // setup cfg based on serial or ethernet
    // no default because modem_start will catch if no connection type is set and print debug statement
    switch(cfg.connection_type())
    {
        case protobuf::DriverConfig::CONNECTION_SERIAL:
            driver_cfg_.set_line_delimiter(SERIAL_DELIMITER);

            if (!cfg.has_serial_baud())
                driver_cfg_.set_serial_baud(DEFAULT_BAUD);

            break;

        case protobuf::DriverConfig::CONNECTION_TCP_AS_CLIENT:
            driver_cfg_.set_line_delimiter(ETHERNET_DELIMITER);

            if(!cfg.has_tcp_server())
                driver_cfg_.set_tcp_server(DEFAULT_TCP_SERVER);

            if(!cfg.has_tcp_port())
                driver_cfg_.set_tcp_port(DEFAULT_TCP_PORT);

            break;

    }

    modem_start(driver_cfg_);

    //Do we need to have specific cfg commands to setup the Evologics?
    write_cfg();

    startup_done_ = true;
}

void goby::acomms::EvologicsDriver::shutdown()
{
    ModemDriverBase::modem_close();
}

void goby::acomms::EvologicsDriver::write_cfg()
{

    //set source level to minimum for testing purposes
    write_single_cfg("L3");

    //set gain level to minimum for testing purposes
    write_single_cfg("G1");

}

void goby::acomms::EvologicsDriver::write_single_cfg(const std::string& s)
{
    std::string cfg_msg = "+++AT!";
    cfg_msg.append(s);

    append_to_write_queue(cfg_msg);
}

void goby::acomms::EvologicsDriver::do_work()
{

    // read any incoming messages from the modem
    std::string in;
    while (modem_read(&in))
    {
        boost::trim(in);

        glog.is(DEBUG1) && glog << group(glog_out_group()) << "Received: " << in.c_str() << std::endl;


        // try to handle the received message, posting appropriate signals
        try
        {
            if(in[0] == '+')
            {
                // process return from an AT message
                // ATSentence recv(in);
                // process_ATreceive(recv);
            }
            else
            {
                // process return from burst data
                process_receive(in);
            }
        }
        catch (std::exception& e)
        {
            glog.is(DEBUG1) && glog << group(glog_in_group()) << warn
                                    << "Failed to handle message: " << e.what() << std::endl;
        }   
    }

}

// void goby::acomms::EvologicsDriver::process_ATreceive(const ATSentence& s)
// {
//     protobuf::ModemRaw raw_msg;
//     raw_msg.set_raw(s.message());
    
//     signal_raw_incoming(raw_msg);

//     glog.is(DEBUG1) && glog << group(glog_in_group()) << raw_msg.raw() << std::endl;
                

//     switch(sentence_id_map_[s.sentence_id()])
//     {
//         case USBLLLONG: break;


//         default: break;




//     }


// }

void goby::acomms::EvologicsDriver::process_receive(const std::string s)
{
    protobuf::ModemRaw raw_msg;
    raw_msg.set_raw(s);

    signal_raw_incoming(raw_msg);

    receive_msg_.add_frame(hex_decode(s));

    signal_receive_and_clear(&receive_msg_);
    

} 

void goby::acomms::EvologicsDriver::append_to_write_queue(const std::string s)
{
    out_.push_back(s);
    try_send(); // try to push it now without waiting for the next call to do_work();
}

void goby::acomms::EvologicsDriver::try_send()
{
    if (out_.empty())
        return;

    const std::string msg = out_.front();


    std::cout << "Writing to driver: " << msg.c_str() << std::endl;

    evologics_write(msg);

    out_.pop_front();
}

void goby::acomms::EvologicsDriver::handle_initiate_transmission(const protobuf::ModemTransmission& msg)
{
    transmit_msg_.CopyFrom(msg);

    try
    {
        glog.is(DEBUG1) && glog << group(glog_out_group()) << "We were asked to transmit from "
                                << msg.src() << " to " << msg.dest() << std::endl;

        signal_modify_transmission(&transmit_msg_);

        switch(transmit_msg_.type())
        {
            case protobuf::ModemTransmission::DATA: data_transmission(&transmit_msg_); break;

            default:
                glog.is(DEBUG1) && glog << group(glog_out_group()) << warn
                                        << "Not initiating transmission because we were given an "
                                           "invalid transmission type for the base Driver:"
                                        << transmit_msg_ << std::endl;
                break;
        }
    }
    catch (ModemDriverException& e)
    {
        glog.is(DEBUG1) && glog << group(glog_out_group()) << warn
                                << "Failed to initiate transmission: " << e.what() << std::endl;
    }
}

void goby::acomms::EvologicsDriver::data_transmission(protobuf::ModemTransmission* msg)
{
    glog.is(DEBUG1) && glog << group(glog_out_group()) << "\tthis is a DATA transmission"
                        << std::endl;

    //do we need to set max frames and bytes for data mode?
    msg->set_max_num_frames(1);
    msg->set_max_frame_bytes(64);

    const bool is_local_cycle = msg->src() == driver_cfg_.modem_id();

    std::cout << "Local Cycle: " << is_local_cycle  << "\tMsg Frame Size: " << msg->frame_size() << std::endl;

    if(!(is_local_cycle && (msg->frame_size() == 0 || msg->frame(0) == "")))
    {
        std::string outgoing = hex_encode(msg->frame(0)+"\r");

        append_to_write_queue(outgoing);

    }
    else
    {
        glog.is(DEBUG1) && glog << group(glog_out_group())
                                << "Not initiating transmission because we have no data to send"
                                << std::endl;
    }
}

void goby::acomms::EvologicsDriver::evologics_write(const std::string s)
{
    protobuf::ModemRaw raw_msg;
    raw_msg.set_raw(s);

    glog.is(DEBUG1) && glog << group(glog_out_group()) << raw_msg.raw() << std::endl;
                            
    signal_raw_outgoing(raw_msg);

    modem_write(raw_msg.raw() + "\r");
}

void goby::acomms::EvologicsDriver::signal_receive_and_clear(protobuf::ModemTransmission* message)
{
    try
    {
        signal_receive(*message);
        message->Clear();
    }
    catch (std::exception& e)
    {
        message->Clear();
        throw;
    }
}