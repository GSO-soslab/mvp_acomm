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
#include <goby/acomms/dccl.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/debug_logger.h>
#include "../proto/goby_msgs.pb.h"
#include "seatrac_driver.h"


void handle_data_receive(const google::protobuf::Message& data_msg);

int main(int argc, char* argv[])
{

    //
    // 1. Create and initialize the driver we want
    //
    goby::acomms::DCCLCodec* dccl_ = goby::acomms::DCCLCodec::get();
    goby::acomms::QueueManager q_manager;
    goby::acomms::SeatracDriver st_driver;
    goby::acomms::MACManager mac;

    goby::acomms::bind(st_driver, q_manager, mac);

    // set the source id of this modem
    uint32_t our_id = 15;
    uint32_t dest_id = 1;

    //Initiate DCCL
    goby::acomms::protobuf::DCCLConfig dccl_cfg;

    dccl_->validate<Pose>();

    //Initiate queue manager
    goby::acomms::protobuf::QueueManagerConfig q_manager_cfg;
    q_manager_cfg.set_modem_id(our_id);


    goby::acomms::connect(&q_manager.signal_receive, &handle_data_receive);

    //Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;
    driver_cfg.set_modem_id(our_id);
    driver_cfg.set_serial_port("/dev/ttyUSB0");
    driver_cfg.set_connection_type(goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);

    //Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(our_id);

    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(our_id);
    my_slot.set_dest(dest_id);
    my_slot.set_rate(0);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    my_slot.set_slot_seconds(5);

    goby::acomms::protobuf::ModemTransmission buddy_slot;
    buddy_slot.set_src(dest_id);
    buddy_slot.set_dest(our_id);
    buddy_slot.set_rate(0);
    buddy_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    buddy_slot.set_slot_seconds(5);

    if (our_id < dest_id)
    {
        mac_cfg.add_slot()->CopyFrom(my_slot);
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
    }
    else
    {
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
        mac_cfg.add_slot()->CopyFrom(my_slot);
    }

    //init protobuf messages to the queue
    q_manager_cfg.add_message_entry()->set_protobuf_name("Pose");
    q_manager_cfg.add_message_entry()->set_protobuf_name("Health");



    goby::glog.set_name("usbl");
    goby::glog.add_stream(goby::util::logger::DEBUG2, &std::clog);

    dccl_->set_cfg(dccl_cfg);
    q_manager.set_cfg(q_manager_cfg);   
    mac.startup(mac_cfg);
    st_driver.startup(driver_cfg);

    Pose p_msg;
    p_msg.set_destination(1);
    p_msg.set_cmd_resp(true);
    q_manager.push_message(p_msg);

    Health h_msg;
    h_msg.set_destination(1);
    h_msg.set_cmd_resp(true);
    q_manager.push_message(h_msg);
    
    // 4. Run the driver
    // 10 hz is good
    int i = 0;
    while (1)
    {
        ++i;
        st_driver.do_work();
        q_manager.do_work();
        mac.do_work();

        // // send another transmission every 10 seconds
        if (!(i % 500))
            q_manager.push_message(p_msg);
            q_manager.push_message(h_msg);

        //mac.do_work();
        usleep(100000);
    }

    return 0;
}

//
// 5. Post the received data
//

void handle_data_receive(const google::protobuf::Message& data_msg)
{
    //std::cout << "got a message: " << data_msg << std::endl;
}