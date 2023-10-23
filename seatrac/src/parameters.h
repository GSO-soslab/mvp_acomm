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

#ifndef ALPHA_DRIVER_PARAMETERS_H
#define ALPHA_DRIVER_PARAMETERS_H

#include <string>

struct SerialParam {
    std::string port;
    int baud;
    int timeout;
    int parity;
    int databits;
    int stopbits;
};

#endif // ALPHA_DRIVER_PARAMETERS_H