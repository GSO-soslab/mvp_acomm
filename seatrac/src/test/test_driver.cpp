// Copyright 2009-2018 Toby Schneider (http://gobysoft.org/index.wt/people/toby)
//                     GobySoft, LLC (2013-)
//                     Massachusetts Institute of Technology (2007-2014)
//
//
// This file is part of the Goby Underwater Autonomy Project Binaries
// ("The Goby Binaries").
//
// The Goby Binaries are free software: you can redistribute them and/or modify
// them under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
//
// The Goby Binaries are distributed in the hope that they will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Goby.  If not, see <http://www.gnu.org/licenses/>.

//
// Usage (WHOI Micro-Modem): run
// > driver_simple /dev/tty_of_modem_A 1
//
// wait a few seconds
//
// > driver_simple /dev/tty_of_modem_B 2
//
// be careful of collisions if you start them at the same time (this is why libamac exists!)

// Usage (example ABCModem): run
// > driver_simple /dev/tty_of_modem_A 1 ABCDriver
// > driver_simple /dev/tty_of_modem_B 2 ABCDriver
// Also see abc_modem_simulator.cpp

#include <iostream>

#include <goby/acomms/bind.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/connect.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>
#include "../proto/goby_msgs.pb.h"
#include "seatrac_driver.h"

#include "../io.h"

void handle_data_receive(const goby::acomms::protobuf::ModemTransmission& data_msg);

int main(int argc, char* argv[])
{

    //
    // 1. Create and initialize the driver we want
    //
    goby::acomms::ModemDriverBase* driver = 0;
    goby::acomms::protobuf::DriverConfig cfg;
    goby::acomms::protobuf::QueueManagerConfig qcfg;
    goby::acomms::QueueManager q_manager;
    goby::acomms::protobuf::ModemTransmission request_msg;
    goby::acomms::MACManager mac;
    goby::acomms::protobuf::MACConfig mcfg;

    

    // set the serial port given on the command line
    cfg.set_serial_port("/dev/ttyUSB0");
    cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    // set the source id of this modem
    uint32_t our_id = goby::util::as<uint32_t>(15);
    uint32_t dest_id = goby::util::as<uint32_t>(1);

    cfg.set_modem_id(our_id);
    qcfg.set_modem_id(our_id);
    mcfg.set_modem_id(our_id);

    //init protobuf messages to the queue
    qcfg.add_message_entry()->set_protobuf_name("Pose");
    qcfg.add_message_entry()->set_protobuf_name("Health");
    qcfg.add_message_entry()->set_protobuf_name("RelativePose");
    qcfg.add_message_entry()->set_protobuf_name("ControllerInfo");
    qcfg.add_message_entry()->set_protobuf_name("DirectControl");
    qcfg.add_message_entry()->set_protobuf_name("StateInfo");
    qcfg.add_message_entry()->set_protobuf_name("SingleWaypoint");
    qcfg.add_message_entry()->set_protobuf_name("MultiWaypoint");
    qcfg.add_message_entry()->set_protobuf_name("ExecuteWaypoint");

    q_manager.set_cfg(qcfg);

    //configure mac slots
    mcfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    goby::acomms::protobuf::ModemTransmission* slot = mcfg.add_slot();
    slot->set_src(our_id);
    slot->set_rate(0);
    slot->set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    slot->set_slot_seconds(10);


    goby::glog.set_name("usbl");
    goby::glog.add_stream(goby::util::logger::DEBUG2, &std::clog);

    std::cout << "Starting seatrac driver: " << std::endl;
    driver = new goby::acomms::SeatracDriver;

    goby::acomms::connect(&driver->signal_receive, &handle_data_receive);
    //goby::acomms::connect(&mac.signal_initiate_transmission, &driver->handle_initiate_transmission);
    goby::acomms::bind(*driver, q_manager, mac);
    
    StateInfo r_out;
    r_out.set_destination(1);
    r_out.set_time(1682531200);
    r_out.set_setget(false);


    q_manager.push_message(r_out);
    //
    // 2. Startup the driver
    //
    driver->startup(cfg);
    mac.startup(mcfg);


    //
    // 3. Initiate a transmission cycle with some data
    //

    // request_msg.set_max_frame_bytes(31);
    // request_msg.set_max_num_frames(1);
    // request_msg.set_dest(1);

    // q_manager.handle_modem_data_request(&request_msg);

    // driver->handle_initiate_transmission(request_msg);

    //
    // 4. Run the driver
    // 10 hz is good
    int i = 0;
    while (1)
    {
        // ++i;
        // driver->do_work();

        // // send another transmission every 10 seconds
        // if (!(i % 10))
        //     driver->handle_initiate_transmission(request_msg);

        // // in here you can initiate more transmissions as you want
        // usleep(100000);

        mac.do_work();
        usleep(100000);
    }

    delete driver;
    return 0;
}

//
// 5. Post the received data
//

void handle_data_receive(const goby::acomms::protobuf::ModemTransmission& data_msg)
{
    std::cout << "got a message: " << data_msg << std::endl;
}