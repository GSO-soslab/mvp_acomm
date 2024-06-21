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
#include "goby/acomms/protobuf/modem_message.pb.h"  // for ModemTransmission
#include "goby/time/system_clock.h"                 // for SystemClock, Sys...

#include "../AT/ATsentence.h"

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

    typedef std::function<void(UsbllongMsg)> UsblCallback;
    UsblCallback usbl_callback_;

    /// \brief Default constructor.
    EvologicsDriver();

    /// Destructor.
    ~EvologicsDriver() override;

    /// \brief Starts the driver.
    ///
    /// \param cfg Configuration for the Micro-Modem driver. DriverConfig is defined in acomms_driver_base.proto.
    void startup(const protobuf::DriverConfig& cfg) override;

    void clear_buffer();

    /// \brief Stops the driver.
    void shutdown() override;

    /// \brief See ModemDriverBase::do_work()
    void do_work() override;

    /// \brief See ModemDriverBase::handle_initiate_transmission()
    void handle_initiate_transmission(const protobuf::ModemTransmission& m) override;

    bool is_started() const { return startup_done_; }

    void set_source_level(int source_level);

    void set_source_control(int source_control);

    void set_gain(int gain);

    void set_carrier_waveform_id(int id);

    void set_local_address(int address);

    void set_remote_address(int address);

    void set_highest_address(int address);

    void set_cluster_size(int size);

    void set_packet_time(int time);

    void set_retry_count(int count);

    void set_retry_timeout(int time);

    void set_keep_online_count(int count);

    void set_idle_timeout(int time);

    void set_channel_protocol_id(int id);

    void set_sound_speed(int speed);

    void set_usbl_callback(UsblCallback c) { usbl_callback_  = c;}

    ATsentence at_sentence;

    // output
    void try_send(); // try to send another NMEA message to the modem
    void append_to_write_queue(const AtType s); // add a message
    void evologics_write(const std::string &s); // actually write a message
    void data_transmission(protobuf::ModemTransmission* msg);

    // input
    void process_receive(const std::string& in); // parse a receive message and call proper method
    void process_at_receive(const std::string& in);

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
    std::deque<AtType> out_;

    // set after the startup routines finish once. we can't startup on instantiation because
    // the base class sets some of our references (from the MOOS file)
    bool startup_done_{false};

    protobuf::ModemTransmission transmit_msg_;
    protobuf::ModemTransmission receive_msg_;

    std::unique_ptr<dccl::Codec> dccl_;
    // DCCL requires full memory barrier...
    static std::mutex dccl_mutex_;

};
} // namespace acomms
} // namespace goby
#endif