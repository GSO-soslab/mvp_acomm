/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller
    Email: jason_miller@uri.edu
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/

#pragma once

#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>


#include <geographic_msgs/GeoPose.h>
#include <mvp_acomms/EvologicsUsbllong.h>



#include <mutex>


class AcommsLocalization{

public:

    AcommsLocalization();
    ~AcommsLocalization();

    void shipGeoposeCallback(const geographic_msgs::GeoPoseConstPtr &msg);
    void usblCallback(const mvp_acomms::EvologicsUsbllongConstPtr &msg);

private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;

    ros::Subscriber ship_geopose_sub_;
    ros::Subscriber evologics_usbl_sub_;

    ros::Publisher usbl_orientation_pub_;

    geographic_msgs::GeoPose ship_geopose_;

    std::mutex ship_geopose_mutex_;

    bool external_heading_enabled_;

};

