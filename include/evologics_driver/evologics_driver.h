// Copyright 2009-2021:
//   GobySoft, LLC (2013-)
//   Massachusetts Institute of Technology (2007-2014)
//   Community contributors (see AUTHORS file)
// File authors:
//   Toby Schneider <toby@gobysoft.org>
//   Russ Webber <russ@rw.id.au>
//
//
// This file is part of the Goby Underwater Autonomy Project Libraries
// ("The Goby Libraries").
//
// The Goby Libraries are free software: you can redistribute them and/or modify
// them under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 2.1 of the License, or
// (at your option) any later version.
//
// The Goby Libraries are distributed in the hope that they will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Goby.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GOBY_ACOMMS_MODEMDRIVER_EVO_DRIVER_H
#define GOBY_ACOMMS_MODEMDRIVER_EVO_DRIVER_H

#include <cstdint> // for uint32_t
#include <deque>    // for deque
#include <map>      // for map
#include <memory>   // for unique_ptr
#include <mutex>    // for mutex
#include <set>      // for set
#include <string>   // for string

#include "goby/acomms/modemdriver/driver_base.h"    // for ModemDriverBase
#include "goby/acomms/protobuf/driver_base.pb.h"    // for DriverConfig
#include "goby/acomms/protobuf/mm_driver.pb.h"      // for Config, MessageT...
#include "goby/acomms/protobuf/modem_message.pb.h"  // for ModemTransmission
#include "goby/time/system_clock.h"                 // for SystemClock, Sys...
#include "goby/util/linebasedcomms/nmea_sentence.h" // for NMEASentence

namespace dccl
{
class Codec;
} // namespace dccl

namespace goby
{
namespace acomms
{
/// \class EvologicsDriver evologics_driver.h goby/acomms/modem_driver.h
/// \ingroup acomms_api
/// \brief provides an API to the Evologics Modem driver
class EvologicsDriver : public ModemDriverBase
{
  public:
    /// \brief Default constructor.
    EvologicsDriver();
    /// Destructor.
    ~EvologicsDriver() override;

    /// \brief Starts the driver.
    ///
    /// \param cfg Configuration for the Micro-Modem driver. DriverConfig is defined in acomms_driver_base.proto, and various extensions specific to the WHOI Micro-Modem are defined in acomms_mm_driver.proto.
    void startup(const protobuf::DriverConfig& cfg) override;

    /// \brief Stops the driver.
    void shutdown() override;

    /// \brief See ModemDriverBase::do_work()
    void do_work() override;

    /// \brief See ModemDriverBase::handle_initiate_transmission()
    void handle_initiate_transmission(const protobuf::ModemTransmission& m) override;

    bool is_started() const { return startup_done_; }

    void write_single_cfg(const std::string& s); // write a single NVRAM value

  private:
    enum SentenceIDs
    {
        SENTENCE_NOT_DEFINED = 0,
        OK,
        DELIVEREDIM,
        RECVIM,
        SENDSTART,
        SENDEND,
        RECVSTART,
        RECVEND,
        FAILEDIM,
        CANCELLEDIM,
        CANCELLEDIMS,
        CANCELLEDPBM,
        EXPIREDIMS,
        BITRATE,
        SRCLEVEL,
        STATUS,
        PHYON,
        PHYOFF,
        RECVFAILED,
        RECVSRV,
        RADDR,
        USBLPHYP,
        USBLPHYD,
        USBLLONG,
        USBLANGLES,
        ERROR,
        RECVPBM,
        RECVIMS  
    };

    // startup
    void initialize_talkers(); // insert strings into sentence_id_map_, etc for later use
    void write_cfg();          // write the NVRAM configuration values to the modem

    // output

    void try_send(); // try to send another NMEA message to the modem
    void append_to_write_queue(const std::string s); // add a message
    void evologics_write(const std::string s); // actually write a message
    void data_transmission(protobuf::ModemTransmission* msg);

    // input
    void process_receive(const std::string s); // parse a receive message and call proper method

    // data cycle
    void handle_ack(std::uint32_t src, std::uint32_t dest, std::uint32_t frame,
                    protobuf::ModemTransmission* m);

    void signal_receive_and_clear(protobuf::ModemTransmission* message);

  private:
    // for the serial connection

    std::string DEFAULT_TCP_SERVER = "192.168.0.209";
    int DEFAULT_TCP_PORT = 9200;
    int DEFAULT_BAUD = 19200;

    static const std::string SERIAL_DELIMITER;
    static const std::string ETHERNET_DELIMITER;

    // all startup configuration (DriverConfig defined in acomms_driver_base.proto and extended in acomms_mm_driver.proto)
    protobuf::DriverConfig driver_cfg_;

    // deque for outgoing messages to the modem, we queue them up and send
    // as the modem acknowledges them
    std::deque<std::string> out_;

    // set after the startup routines finish once. we can't startup on instantiation because
    // the base class sets some of our references (from the MOOS file)
    bool startup_done_{false};

    std::map<std::string, SentenceIDs> sentence_id_map_;

    protobuf::ModemTransmission transmit_msg_;
    protobuf::ModemTransmission receive_msg_;

    std::unique_ptr<dccl::Codec> dccl_;
    // DCCL requires full memory barrier...
    static std::mutex dccl_mutex_;
};
} // namespace acomms
} // namespace goby
#endif