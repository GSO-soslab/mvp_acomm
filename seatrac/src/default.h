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

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2023-2023

    Copyright (C) 2023-2023 Smart Ocean Systems Laboratory
*/

#ifndef ALPHA_DRIVER_DEFAULT_H
#define ALPHA_DRIVER_DEFAULT_H

/*****************************************************************************/
/***                                 Serial                                ***/
/*****************************************************************************/
#define DEFAULT_PORT "/dev/ttyUSB0"      // Default port name
#define DEFAULT_BAUD 115200              // Default baudate
#define DEFAULT_PARITY 0                 // Default parity: parity_none
#define DEFAULT_DATABITS 8               // Default databits
#define DEFAULT_STOPBITS 1               // Default stopbits
#define DEFAULT_TIMEOUT 1000             // Default timeout


#define DEFAULT_CONSOLE_DEBUG false
#define DEFAULT_BEACON_ID 0
/*****************************************************************************/
/***                               Thruster                                ***/
/*****************************************************************************/

#endif // ALPHA_DRIVER_DEFAULT_H